#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <spdlog/spdlog.h>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {
namespace {

using Clock = std::chrono::steady_clock;

struct FineScore {
  double overlap_q2m = 0.0;
  double overlap_m2q = 0.0;
  double trimmed_rmse = std::numeric_limits<double>::infinity();
  double degeneracy_ratio = 0.0;
  double quality = -std::numeric_limits<double>::infinity();
};

double yaw_from_pose(const Eigen::Isometry3f& pose) {
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Isometry3f project_to_se2(const Eigen::Isometry3f& pose) {
  const float yaw = static_cast<float>(yaw_from_pose(pose));
  Eigen::Isometry3f projected = Eigen::Isometry3f::Identity();
  projected.translation().x() = pose.translation().x();
  projected.translation().y() = pose.translation().y();
  projected.linear() = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return projected;
}

bool same_pose_cluster(
  const Eigen::Isometry3f& lhs,
  const Eigen::Isometry3f& rhs,
  const FineScoringParams& params) {
  const double translation =
    (lhs.translation().head<2>() - rhs.translation().head<2>()).norm();
  const double yaw_distance = std::abs(wrap_angle(
    yaw_from_pose(lhs) - yaw_from_pose(rhs)));
  return translation <= params.cluster_translation_m &&
         yaw_distance <= params.cluster_yaw_rad;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_local_map(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map,
  const Eigen::Isometry3f& candidate,
  double radius,
  int target_max_points) {
  auto local = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  local->reserve(global_map->size());
  const float center_x = candidate.translation().x();
  const float center_y = candidate.translation().y();
  const float radius_sq = static_cast<float>(radius * radius);
  for (const auto& point : *global_map) {
    const float dx = point.x - center_x;
    const float dy = point.y - center_y;
    if (dx * dx + dy * dy <= radius_sq) {
      local->push_back(point);
    }
  }

  if (target_max_points > 0 &&
      local->size() > static_cast<std::size_t>(target_max_points)) {
    const double scale = std::sqrt(
      static_cast<double>(local->size()) / target_max_points);
    const float leaf = static_cast<float>(std::max(0.10, 0.10 * scale));
    auto reduced = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(local);
    voxel.filter(*reduced);
    local = reduced;
  }
  return local;
}

bool run_se2_icp_stage(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target,
  const Eigen::Matrix4f& initial_guess,
  double max_correspondence,
  int maximum_iterations,
  Eigen::Matrix4f* output,
  std::string* status) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  typename pcl::registration::TransformationEstimation2D<
    pcl::PointXYZ, pcl::PointXYZ>::Ptr estimation(
      new pcl::registration::TransformationEstimation2D<
        pcl::PointXYZ, pcl::PointXYZ>());
  icp.setTransformationEstimation(estimation);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(max_correspondence);
  icp.setMaximumIterations(maximum_iterations);
  icp.setTransformationEpsilon(1e-7);
  icp.setEuclideanFitnessEpsilon(1e-6);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, initial_guess);
  if (!icp.hasConverged()) {
    *status = "icp_not_converged";
    return false;
  }

  *output = icp.getFinalTransformation();
  if (!output->allFinite()) {
    *status = "icp_non_finite";
    return false;
  }
  *status = "converged";
  return true;
}

FineScore score_candidate(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr query,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr local_map,
  const Eigen::Isometry3f& pose,
  const FineScoringParams& params) {
  FineScore score;
  if (!query || !local_map || query->empty() || local_map->empty()) {
    return score;
  }

  auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*query, *transformed, pose.matrix());

  pcl::KdTreeFLANN<pcl::PointXYZ> map_tree;
  map_tree.setInputCloud(local_map);
  const double max_distance_sq =
    params.fine_correspondence_distance *
    params.fine_correspondence_distance;
  std::vector<double> accepted_squared_distances;
  accepted_squared_distances.reserve(transformed->size());
  Eigen::Matrix3d normal_matrix = Eigen::Matrix3d::Zero();
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto& point : *transformed) {
    if (map_tree.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
        squared_distances[0] <= max_distance_sq) {
      accepted_squared_distances.push_back(squared_distances[0]);
      const double x = point.x;
      const double y = point.y;
      Eigen::Matrix<double, 2, 3> jacobian;
      jacobian << 1.0, 0.0, -y,
                  0.0, 1.0,  x;
      normal_matrix.noalias() += jacobian.transpose() * jacobian;
    }
  }
  score.overlap_q2m = static_cast<double>(accepted_squared_distances.size()) /
                      transformed->size();

  if (!accepted_squared_distances.empty()) {
    std::sort(accepted_squared_distances.begin(), accepted_squared_distances.end());
    const std::size_t keep_count = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(
        accepted_squared_distances.size() * params.fine_trim_fraction)));
    const double sum = std::accumulate(
      accepted_squared_distances.begin(),
      accepted_squared_distances.begin() + keep_count,
      0.0);
    score.trimmed_rmse = std::sqrt(sum / keep_count);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(normal_matrix);
    if (eigen_solver.info() == Eigen::Success) {
      const auto eigenvalues = eigen_solver.eigenvalues();
      const double largest = eigenvalues.maxCoeff();
      if (largest > 0.0) {
        score.degeneracy_ratio =
          std::max(0.0, eigenvalues.minCoeff()) / largest;
      }
    }
  }

  double min_range = std::numeric_limits<double>::infinity();
  double max_range = 0.0;
  for (const auto& point : *query) {
    const double range = std::hypot(point.x, point.y);
    min_range = std::min(min_range, range);
    max_range = std::max(max_range, range);
  }
  const double visibility_margin = params.fine_correspondence_distance;
  pcl::KdTreeFLANN<pcl::PointXYZ> query_tree;
  query_tree.setInputCloud(transformed);
  std::size_t visible_map_points = 0;
  std::size_t matched_map_points = 0;
  const Eigen::Isometry3f inverse = pose.inverse();
  for (const auto& map_point : *local_map) {
    const Eigen::Vector3f local_point = inverse * map_point.getVector3fMap();
    const double range = std::hypot(local_point.x(), local_point.y());
    if (range + visibility_margin < min_range ||
        range > max_range + visibility_margin) {
      continue;
    }
    ++visible_map_points;
    if (query_tree.nearestKSearch(map_point, 1, indices, squared_distances) == 1 &&
        squared_distances[0] <= max_distance_sq) {
      ++matched_map_points;
    }
  }
  if (visible_map_points > 0) {
    score.overlap_m2q = static_cast<double>(matched_map_points) /
                        visible_map_points;
  }

  if (std::isfinite(score.trimmed_rmse)) {
    score.quality = 0.5 * (score.overlap_q2m + score.overlap_m2q) -
                    score.trimmed_rmse;
  }
  return score;
}

}  // namespace

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC(
  const GlobalLocalizationEngineFPFH_RANSACParams& params)
: params(params) {}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr
GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  const auto total_started = Clock::now();
  const double normal_estimation_radius = params.normal_estimation_radius;
  const double search_radius = params.search_radius;

  spdlog::info("Normal Estimation: Radius({})", normal_estimation_radius);
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);
  const auto normal_finished = Clock::now();

  const auto features = extract_fpfh(cloud, cloud, normals);
  const auto feature_finished = Clock::now();
  spdlog::info(
    "FPFH timing: points={} normal_sec={:.6f} fpfh_sec={:.6f} total_sec={:.6f}",
    cloud->size(),
    std::chrono::duration<double>(normal_finished - total_started).count(),
    std::chrono::duration<double>(feature_finished - normal_finished).count(),
    std::chrono::duration<double>(feature_finished - total_started).count());
  return features;
}

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr
GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr surface,
  pcl::PointCloud<pcl::Normal>::ConstPtr surface_normals) {
  if (!keypoints || !surface || !surface_normals || keypoints->empty() ||
      surface->empty() || surface->size() != surface_normals->size()) {
    throw std::invalid_argument("invalid keypoint/surface/normal inputs for FPFH");
  }

  spdlog::info(
    "FPFH Extraction: keypoints={} surface={} search_radius={}",
    keypoints->size(), surface->size(), params.search_radius);
  auto features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  pcl::FPFHEstimationOMP<
    pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(params.search_radius);
  fest.setInputCloud(keypoints);
  fest.setSearchSurface(surface);
  fest.setInputNormals(surface_normals);
  fest.compute(*features);
  return features;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  global_map = cloud;
  global_map_keypoints = cloud;
  global_map_features = extract_fpfh(cloud);

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(
    params.ransac_params));
  ransac->set_target(global_map, global_map_features);
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map_cache(
  const MapFeatureCache::Ptr& cache) {
  if (!cache || !cache->surface || !cache->keypoints || !cache->features) {
    throw std::invalid_argument("invalid map feature cache");
  }
  global_map = cache->surface;
  global_map_keypoints = cache->keypoints;
  global_map_features = cache->features;

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(
    params.ransac_params));
  ransac->set_target(global_map_keypoints, global_map_features, global_map);
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query_coarse(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints,
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr features,
  int candidate_pool) {
  if (!ransac) {
    throw std::runtime_error("global map has not been configured");
  }
  ransac->set_source(keypoints, features);
  auto results = ransac->estimate();
  results.sort(std::max(1, candidate_pool));
  return results;
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  int max_num_candidates) {
  const auto cloud_features = extract_fpfh(cloud);
  auto results = query_coarse(
    cloud,
    cloud_features,
    std::max(max_num_candidates, params.fine.diagnostic_candidate_count));
  const std::size_t diagnostic_count = std::min<std::size_t>(
    params.fine.diagnostic_candidate_count, results.results.size());
  for (std::size_t i = 0; i < diagnostic_count; ++i) {
    const auto& result = results.results[i];
    const Eigen::Vector3f translation = result->pose.translation();
    spdlog::info(
      "RANSAC top_candidate rank={} x={:.6f} y={:.6f} z={:.6f} "
      "yaw={:.6f} inlier={:.6f} error={:.6f}",
      i + 1, translation.x(), translation.y(), translation.z(),
      yaw_from_pose(result->pose), result->inlier_fraction, result->error);
  }
  const auto limit =
    static_cast<std::size_t>(std::max(0, max_num_candidates));
  if (results.results.size() > limit) {
    results.results.resize(limit);
  }
  return results;
}

DetailedGlobalLocalizationResults
GlobalLocalizationEngineFPFH_RANSAC::query_v2(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr surface,
  pcl::PointCloud<pcl::Normal>::ConstPtr surface_normals,
  int max_num_candidates,
  double matching_deadline_sec) {
  DetailedGlobalLocalizationResults output;
  output.score_profile = params.fine.score_profile;
  const auto started = Clock::now();
  const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(std::max(0.1, matching_deadline_sec)));
  auto finish = [&]() {
    output.total_time_sec =
      std::chrono::duration<double>(Clock::now() - started).count();
    return output;
  };

  if (!params.fine.enabled) {
    output.rejection_reason = "fine_scoring_disabled";
    return finish();
  }
  if (!surface || !surface_normals || surface->empty() ||
      surface->size() != surface_normals->size()) {
    output.rejection_reason = "invalid_query_surface_or_normals";
    return finish();
  }

  auto keypoints = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
  const float leaf = static_cast<float>(params.fine.query_keypoint_voxel);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(surface);
  voxel.filter(*keypoints);
  if (keypoints->size() < 3) {
    output.rejection_reason = "insufficient_query_keypoints";
    return finish();
  }

  const auto features = extract_fpfh(keypoints, surface, surface_normals);
  auto coarse = query_coarse(
    keypoints, features, params.fine.internal_candidate_pool);
  const std::size_t diagnostic_count = std::min<std::size_t>(
    params.fine.diagnostic_candidate_count, coarse.results.size());
  output.diagnostic_coarse.assign(
    coarse.results.begin(), coarse.results.begin() + diagnostic_count);
  if (coarse.results.empty()) {
    output.rejection_reason = "no_coarse_candidates";
    return finish();
  }

  std::vector<GlobalLocalizationResult::Ptr> clustered;
  for (const auto& candidate : coarse.results) {
    const Eigen::Isometry3f projected = project_to_se2(candidate->pose);
    const bool duplicate = std::any_of(
      clustered.begin(), clustered.end(),
      [&](const auto& selected) {
        return same_pose_cluster(
          projected, project_to_se2(selected->pose), params.fine);
      });
    if (!duplicate) {
      clustered.push_back(candidate);
      if (clustered.size() >=
          static_cast<std::size_t>(params.fine.refinement_top_k)) {
        break;
      }
    }
  }

  double query_max_range = 0.0;
  for (const auto& point : *surface) {
    query_max_range = std::max(
      query_max_range,
      std::hypot(static_cast<double>(point.x), point.y));
  }
  const double crop_radius = query_max_range + params.fine.local_map_margin_m;

  for (const auto& coarse_candidate : clustered) {
    DetailedGlobalLocalizationResult detailed;
    detailed.coarse_inlier = coarse_candidate->inlier_fraction;
    detailed.coarse_error = coarse_candidate->error;
    detailed.pose = project_to_se2(coarse_candidate->pose);

    if (Clock::now() >= deadline) {
      detailed.refinement_status = "not_started";
      detailed.rejection_reason = "matching_deadline_exhausted";
      output.candidates.push_back(detailed);
      continue;
    }

    const auto local_map = crop_local_map(
      global_map, detailed.pose, crop_radius, params.fine.target_max_points);
    if (local_map->size() < 3) {
      detailed.refinement_status = "not_started";
      detailed.rejection_reason = "local_map_too_small";
      output.candidates.push_back(detailed);
      continue;
    }

    if (params.fine.se2_refinement_enabled) {
      Eigen::Matrix4f coarse_icp;
      std::string icp_status;
      if (!run_se2_icp_stage(
            surface, local_map, detailed.pose.matrix(),
            params.fine.se2_icp_coarse_correspondence,
            params.fine.se2_icp_coarse_iterations,
            &coarse_icp, &icp_status)) {
        detailed.refinement_status = "coarse_" + icp_status;
        detailed.rejection_reason = detailed.refinement_status;
        output.candidates.push_back(detailed);
        continue;
      }
      if (Clock::now() >= deadline) {
        detailed.refinement_status = "coarse_converged";
        detailed.rejection_reason = "matching_deadline_exhausted";
        output.candidates.push_back(detailed);
        continue;
      }

      Eigen::Matrix4f fine_icp;
      if (!run_se2_icp_stage(
            surface, local_map, coarse_icp,
            params.fine.se2_icp_fine_correspondence,
            params.fine.se2_icp_fine_iterations,
            &fine_icp, &icp_status)) {
        detailed.refinement_status = "fine_" + icp_status;
        detailed.rejection_reason = detailed.refinement_status;
        output.candidates.push_back(detailed);
        continue;
      }

      detailed.pose = project_to_se2(Eigen::Isometry3f(fine_icp));
      detailed.refinement_status = "converged";
    } else {
      detailed.refinement_status = "coarse_only";
    }
    const FineScore score = score_candidate(
      surface, local_map, detailed.pose, params.fine);
    detailed.fine_overlap_q2m = score.overlap_q2m;
    detailed.fine_overlap_m2q = score.overlap_m2q;
    detailed.fine_trimmed_rmse = score.trimmed_rmse;
    detailed.degeneracy_ratio = score.degeneracy_ratio;
    detailed.quality = score.quality;

    if (!params.fine.score_profile_calibrated) {
      detailed.rejection_reason = "score_profile_not_calibrated";
    } else if (score.overlap_q2m < params.fine.min_fine_overlap_q2m) {
      detailed.rejection_reason = "fine_overlap_q2m_below_threshold";
    } else if (score.overlap_m2q < params.fine.min_fine_overlap_m2q) {
      detailed.rejection_reason = "fine_overlap_m2q_below_threshold";
    } else if (!std::isfinite(score.trimmed_rmse) ||
               score.trimmed_rmse > params.fine.max_fine_trimmed_rmse) {
      detailed.rejection_reason = "fine_trimmed_rmse_above_threshold";
    } else if (score.degeneracy_ratio < params.fine.min_degeneracy_ratio) {
      detailed.rejection_reason = "se2_degenerate";
    } else {
      detailed.valid = true;
    }
    output.candidates.push_back(detailed);
  }

  std::sort(
    output.candidates.begin(), output.candidates.end(),
    [](const auto& lhs, const auto& rhs) {
      if (lhs.valid != rhs.valid) {
        return lhs.valid > rhs.valid;
      }
      return lhs.quality > rhs.quality;
    });

  if (output.candidates.size() >= 2) {
    const double margin =
      output.candidates[0].quality - output.candidates[1].quality;
    output.candidates[0].top1_top2_margin = margin;
    if (output.candidates[0].valid &&
        margin < params.fine.min_top1_top2_margin) {
      output.candidates[0].valid = false;
      output.candidates[0].rejection_reason = "top1_top2_ambiguous";
    }
  }

  const auto valid = std::find_if(
    output.candidates.begin(), output.candidates.end(),
    [](const auto& candidate) { return candidate.valid; });
  if (valid == output.candidates.end()) {
    output.rejection_reason = output.candidates.empty()
      ? "no_refined_candidates"
      : output.candidates.front().rejection_reason;
  }
  const auto limit =
    static_cast<std::size_t>(std::max(0, max_num_candidates));
  if (output.candidates.size() > limit) {
    output.candidates.resize(limit);
  }
  return finish();
}

}  // namespace hdl_global_localization
