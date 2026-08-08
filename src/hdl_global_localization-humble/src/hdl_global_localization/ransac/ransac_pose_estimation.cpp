/*
 * This RansacPoseEstimation implementation was written based on pcl::SampleConsensusPrerejective
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <atomic>
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <random>
#include <spdlog/spdlog.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_poly.h>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater_flann.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater_voxels.hpp>

namespace hdl_global_localization {

template <typename FeatureT>
RansacPoseEstimation<FeatureT>::RansacPoseEstimation(const RansacPoseEstimationParams& params) : params(params) {}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features) {
  set_target(target, target_features, target);
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_target(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_keypoints,
  typename pcl::PointCloud<FeatureT>::ConstPtr target_features,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_evaluation_cloud) {
  if (!target_keypoints || !target_features || !target_evaluation_cloud ||
      target_keypoints->empty() || target_evaluation_cloud->empty() ||
      target_keypoints->size() != target_features->size()) {
    throw std::invalid_argument("invalid RANSAC target clouds/features");
  }

  this->target = target_keypoints;
  this->target_features = target_features;
  this->target_evaluation_cloud = target_evaluation_cloud;
  feature_tree.reset(new pcl::KdTreeFLANN<FeatureT>);
  feature_tree->setInputCloud(target_features);

  if (params.voxel_based) {
    evaluater.reset(new MatchingCostEvaluaterVoxels());
  } else {
    evaluater.reset(new MatchingCostEvaluaterFlann());
  }
  evaluater->set_target(target_evaluation_cloud, params.max_correspondence_distance);
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features) {
  this->source = source;
  this->source_features = source_features;
}

template <typename FeatureT>
GlobalLocalizationResults RansacPoseEstimation<FeatureT>::estimate() {
  const auto estimate_started = std::chrono::steady_clock::now();
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;

  pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> correspondence_rejection;
  correspondence_rejection.setInputTarget(target);
  correspondence_rejection.setInputSource(source);
  correspondence_rejection.setCardinality(3);
  correspondence_rejection.setSimilarityThreshold(params.similarity_threshold);

  const auto feature_started = std::chrono::steady_clock::now();
  spdlog::info("RANSAC: precomputing nearest feature correspondences (source_points={})", source->size());
  std::vector<std::vector<int>> similar_features(source->size());
#pragma omp parallel for
  for (int i = 0; i < source->size(); i++) {
    std::vector<float> sq_dists;
    feature_tree->nearestKSearch(source_features->at(i), params.correspondence_randomness, similar_features[i], sq_dists);
  }
  const double feature_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - feature_started).count();

  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(i * 8191 + i + target->size() + source->size());
  }

  spdlog::info("RANSAC: main loop (matching_budget={}, max_iterations={})", params.matching_budget, params.max_iterations);
  std::atomic_int matching_count(0);
  std::atomic_int iterations(0);
  std::vector<GlobalLocalizationResult::Ptr> results(params.max_iterations);
  int matching_budget = params.matching_budget;
  double min_inlier_fraction = params.min_inlier_fraction;

#pragma omp parallel for
  for (int i = 0; i < results.size(); i++) {
    if (matching_count > matching_budget) {
      continue;
    }
    iterations++;

    auto& mt = mts[omp_get_thread_num()];
    std::vector<int> samples;
    std::vector<int> correspondences;
    select_samples(mt, similar_features, samples, correspondences);

    if (!correspondence_rejection.thresholdPolygon(samples, correspondences)) {
      continue;
    }

    Eigen::Matrix4f transformation;
    transformation_estimation.estimateRigidTransformation(*source, samples, *target, correspondences, transformation);

    // if (!params.is_valid(Eigen::Isometry3f(transformation))) {
    //   continue;
    // }

    matching_count++;
    double inlier_fraction = 0.0;
    double matching_error = evaluater->calc_matching_error(*source, transformation, &inlier_fraction);
    if (matching_count > 0 && matching_count % 5000 == 0) {
      spdlog::debug(
        "RANSAC progress: iterations={} matching_count={}",
        iterations.load(), matching_count.load());
    }

    if (inlier_fraction > min_inlier_fraction) {
      results[i].reset(new GlobalLocalizationResult(matching_error, inlier_fraction, Eigen::Isometry3f(transformation)));
    }
  }

  GlobalLocalizationResults output(results);
  const auto valid_count = std::count_if(
    output.results.begin(), output.results.end(),
    [](const auto& result) { return result != nullptr; });
  const double total_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - estimate_started).count();
  spdlog::info(
    "RANSAC summary: source_points={} feature_lookup_sec={:.6f} "
    "iterations={} matching_count={} valid_candidates={} total_sec={:.6f}",
    source->size(), feature_sec, iterations.load(), matching_count.load(),
    valid_count, total_sec);
  return output;
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::select_samples(
  std::mt19937& mt,
  const std::vector<std::vector<int>>& similar_features,
  std::vector<int>& samples,
  std::vector<int>& correspondences) const {
  samples.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    samples[i] = std::uniform_int_distribution<>(0, similar_features.size() - 1)(mt);

    for (int j = 0; j < i; j++) {
      if (samples[j] == samples[i]) {
        i--;
      }
    }
  }

  correspondences.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    if (similar_features[samples[i]].size() == 1) {
      correspondences[i] = similar_features[samples[i]][0];
    } else {
      int idx = std::uniform_int_distribution<>(0, similar_features[samples[i]].size() - 1)(mt);
      correspondences[i] = similar_features[samples[i]][idx];
    }
  }
}

// explicit instantiation
template class RansacPoseEstimation<pcl::FPFHSignature33>;

}  // namespace hdl_global_localization
