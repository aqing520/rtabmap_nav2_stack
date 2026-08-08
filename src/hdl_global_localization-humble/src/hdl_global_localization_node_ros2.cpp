#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/load_global_map_cache.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/query_global_localization_v2.hpp>

#include <hdl_global_localization/util/config.hpp>
#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <hdl_global_localization/cache/map_feature_cache.hpp>

namespace hdl_global_localization {

class GlobalLocalizationNode : public rclcpp::Node {
public:
  GlobalLocalizationNode(rclcpp::NodeOptions& options) : rclcpp::Node("hdl_global_localization_node", options) {
    const std::string config_path = ament_index_cpp::get_package_share_directory("hdl_global_localization") + "/config";
    GlobalConfig::instance(config_path);

    const Config config(GlobalConfig::get_config_path("config_base"));
    globalmap_downsample_resolution = config.param<double>("base", "globalmap_downsample_resolution", 0.5);
    query_downsample_resolution = config.param<double>("base", "query_downsample_resolution", 0.5);

    const Config fpfh_config(GlobalConfig::get_config_path("config_fpfh"));
    const bool structural_profile_enabled =
      fpfh_config.param<bool>("features", "use_structural_profile", false);
    active_fpfh_profile =
      structural_profile_enabled ? "garage_structural_v1" : "legacy_fpfh_v1";
    if (const char* forced_profile = std::getenv("HDL_FPFH_PROFILE")) {
      const std::string requested_profile(forced_profile);
      if (requested_profile == "legacy_fpfh_v1" ||
          requested_profile == "garage_structural_v1") {
        active_fpfh_profile = requested_profile;
      } else if (!requested_profile.empty()) {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Ignoring unsupported HDL_FPFH_PROFILE=" << requested_profile);
      }
    }
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Active FPFH profile: " << active_fpfh_profile);
    const std::vector<std::string> profile_path = {
      "profiles", active_fpfh_profile};
    fpfh_params.normal_estimation_radius =
      fpfh_config.param_nested<double>(
        profile_path, "normal_radius", 0.5);
    fpfh_params.search_radius =
      fpfh_config.param_nested<double>(
        profile_path, "fpfh_radius", 1.5);
    fpfh_params.ransac_params.matching_budget =
      fpfh_config.param_nested<int>(
        profile_path, "coarse_matching_budget", 10000);
    fpfh_params.fine.enabled =
      fpfh_config.param<bool>("features", "enable_fine_scoring", false);
    fpfh_params.fine.se2_refinement_enabled =
      fpfh_config.param<bool>("features", "enable_se2_refinement", false);
    fpfh_params.fine.internal_candidate_pool =
      fpfh_config.param_nested<int>(
        profile_path, "coarse_candidate_pool", 100);
    fpfh_params.fine.diagnostic_candidate_count =
      fpfh_config.param_nested<int>(
        profile_path, "diagnostic_candidate_count", 20);
    fpfh_params.fine.refinement_top_k =
      fpfh_config.param_nested<int>(
        profile_path, "refinement_top_k", 5);
    fpfh_params.fine.target_max_points =
      fpfh_config.param_nested<int>(
        profile_path, "target_max_points", 100000);
    fpfh_params.fine.query_keypoint_voxel =
      fpfh_config.param_nested<double>(
        profile_path, "query_keypoint_voxel", 0.2);
    fpfh_params.fine.cluster_translation_m =
      fpfh_config.param_nested<double>(
        profile_path, "cluster_translation_m", 1.0);
    fpfh_params.fine.cluster_yaw_rad =
      fpfh_config.param_nested<double>(
        profile_path, "cluster_yaw_deg", 15.0) * M_PI / 180.0;
    fpfh_params.fine.local_map_margin_m =
      fpfh_config.param_nested<double>(
        profile_path, "local_map_margin_m", 5.0);
    fpfh_params.fine.fine_correspondence_distance =
      fpfh_config.param_nested<double>(
        profile_path, "fine_correspondence_distance", 0.25);
    fpfh_params.fine.fine_trim_fraction =
      fpfh_config.param_nested<double>(
        profile_path, "fine_trim_fraction", 0.8);
    fpfh_params.fine.se2_icp_coarse_correspondence =
      fpfh_config.param_nested<double>(
        profile_path, "se2_icp_coarse_correspondence", 0.8);
    fpfh_params.fine.se2_icp_coarse_iterations =
      fpfh_config.param_nested<int>(
        profile_path, "se2_icp_coarse_iterations", 20);
    fpfh_params.fine.se2_icp_fine_correspondence =
      fpfh_config.param_nested<double>(
        profile_path, "se2_icp_fine_correspondence", 0.25);
    fpfh_params.fine.se2_icp_fine_iterations =
      fpfh_config.param_nested<int>(
        profile_path, "se2_icp_fine_iterations", 30);
    const std::vector<std::string> score_path = {
      "profiles", active_fpfh_profile, "score_profile"};
    fpfh_params.fine.score_profile =
      fpfh_config.param_nested<std::string>(
        score_path, "name", "unconfigured");
    fpfh_params.fine.score_profile_calibrated =
      fpfh_config.param_nested<bool>(
        score_path, "calibrated", false);
    fpfh_params.fine.min_fine_overlap_q2m =
      fpfh_config.param_nested<double>(
        score_path, "min_fine_overlap_q2m", 0.0);
    fpfh_params.fine.min_fine_overlap_m2q =
      fpfh_config.param_nested<double>(
        score_path, "min_fine_overlap_m2q", 0.0);
    fpfh_params.fine.max_fine_trimmed_rmse =
      fpfh_config.param_nested<double>(
        score_path, "max_fine_trimmed_rmse",
        std::numeric_limits<double>::infinity());
    fpfh_params.fine.min_degeneracy_ratio =
      fpfh_config.param_nested<double>(
        score_path, "min_degeneracy_ratio", 0.0);
    fpfh_params.fine.min_top1_top2_margin =
      fpfh_config.param_nested<double>(
        score_path, "min_top1_top2_margin", 0.0);

    set_engine("FPFH_RANSAC");

    using std::placeholders::_1;
    using std::placeholders::_2;
    set_engine_service = this->create_service<srv::SetGlobalLocalizationEngine>("set_engine", std::bind(&GlobalLocalizationNode::set_global_localization_engine, this, _1, _2));
    set_global_map_service = this->create_service<srv::SetGlobalMap>("set_global_map", std::bind(&GlobalLocalizationNode::set_global_map, this, _1, _2));
    load_global_map_cache_service = this->create_service<srv::LoadGlobalMapCache>(
      "load_global_map_cache",
      std::bind(&GlobalLocalizationNode::load_global_map_cache, this, _1, _2));
    query_service = this->create_service<srv::QueryGlobalLocalization>("query", std::bind(&GlobalLocalizationNode::query, this, _1, _2));
    query_v2_service = this->create_service<srv::QueryGlobalLocalizationV2>(
      "query_v2",
      std::bind(&GlobalLocalizationNode::query_v2, this, _1, _2));
  }

  void set_global_localization_engine(const srv::SetGlobalLocalizationEngine::Request::SharedPtr req, srv::SetGlobalLocalizationEngine::Response::SharedPtr res) {
    set_engine(req->engine_name.data);
  }

  bool set_engine(const std::string& engine_name) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Set Global Localization Engine (" << engine_name << ")");
    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(*this));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(fpfh_params));
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Unknown Global Localization Engine:" << engine_name);
      return false;
    }

    if (global_map) {
      if (engine_name == "FPFH_RANSAC" && global_map_cache) {
        auto* fpfh_engine =
          dynamic_cast<GlobalLocalizationEngineFPFH_RANSAC*>(engine.get());
        fpfh_engine->set_global_map_cache(global_map_cache);
      } else {
        engine->set_global_map(global_map);
      }
    }

    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }

  void set_global_map(const srv::SetGlobalMap::Request::SharedPtr req, srv::SetGlobalMap::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->global_map, *cloud);
    cloud = downsample(cloud, globalmap_downsample_resolution);

    globalmap_header = req->global_map.header;
    global_map = cloud;
    global_map_cache.reset();
    engine->set_global_map(global_map);

    RCLCPP_INFO_STREAM(this->get_logger(), "DONE");
  }

  void load_global_map_cache(
    const srv::LoadGlobalMapCache::Request::SharedPtr req,
    srv::LoadGlobalMapCache::Response::SharedPtr res) {
    const auto started = std::chrono::steady_clock::now();
    auto* fpfh_engine =
      dynamic_cast<GlobalLocalizationEngineFPFH_RANSAC*>(engine.get());
    if (!fpfh_engine) {
      res->success = false;
      res->cache_hit = false;
      res->message = "active engine does not support FPFH map caches";
      return;
    }

    std::string error;
    auto candidate = MapFeatureCache::load(
      req->cache_directory, req->source_pcd_path, &error);
    if (!candidate) {
      res->success = false;
      res->cache_hit = false;
      res->message = error;
      RCLCPP_ERROR_STREAM(this->get_logger(), "Map cache rejected: " << error);
      return;
    }
    if (candidate->profile_name != active_fpfh_profile) {
      res->success = false;
      res->cache_hit = false;
      res->message =
        "cache profile " + candidate->profile_name +
        " does not match active profile " + active_fpfh_profile;
      return;
    }

    const auto index_started = std::chrono::steady_clock::now();
    std::unique_ptr<GlobalLocalizationEngineFPFH_RANSAC> candidate_engine;
    try {
      candidate_engine =
        std::make_unique<GlobalLocalizationEngineFPFH_RANSAC>(fpfh_params);
      candidate_engine->set_global_map_cache(candidate);
    } catch (const std::exception& exception) {
      res->success = false;
      res->cache_hit = false;
      res->message = exception.what();
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Map cache index/evaluator construction failed; active map preserved: "
          << exception.what());
      return;
    }
    const double index_and_evaluator_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - index_started).count();

    engine = std::move(candidate_engine);
    global_map_cache = candidate;
    global_map = candidate->surface;
    globalmap_header.frame_id = "map";
    globalmap_header.stamp = this->now();

    res->success = true;
    res->cache_hit = true;
    res->message = "cache loaded and activated";
    res->cache_key = candidate->cache_key;
    res->surface_points = candidate->surface->size();
    res->keypoint_points = candidate->keypoints->size();
    res->cache_load_sec = candidate->artifact_load_sec;
    res->feature_index_sec = index_and_evaluator_sec;
    res->evaluator_build_sec = 0.0;

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Map cache activated key=" << candidate->cache_key
      << " surface_points=" << candidate->surface->size()
      << " keypoints=" << candidate->keypoints->size()
      << " artifact_load_sec=" << candidate->artifact_load_sec
      << " index_evaluator_sec=" << index_and_evaluator_sec
      << " total_sec=" << std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count());
  }

  void query(const srv::QueryGlobalLocalization::Request::SharedPtr req, srv::QueryGlobalLocalization::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Query Global Localization");
    if (global_map == nullptr) {
      RCLCPP_WARN_STREAM(this->get_logger(), "No Globalmap");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->cloud, *cloud);
    cloud = downsample(cloud, query_downsample_resolution);

    auto results = engine->query(cloud, req->max_num_candidates);

    res->inlier_fractions.resize(results.results.size());
    res->errors.resize(results.results.size());
    res->poses.resize(results.results.size());

    res->header = req->cloud.header;
    res->globalmap_header = globalmap_header;

    for (int i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res->inlier_fractions[i] = result->inlier_fraction;
      res->errors[i] = result->error;
      res->poses[i].orientation.x = quat.x();
      res->poses[i].orientation.y = quat.y();
      res->poses[i].orientation.z = quat.z();
      res->poses[i].orientation.w = quat.w();

      res->poses[i].position.x = trans.x();
      res->poses[i].position.y = trans.y();
      res->poses[i].position.z = trans.z();
    }
  }

  void query_v2(
    const srv::QueryGlobalLocalizationV2::Request::SharedPtr req,
    srv::QueryGlobalLocalizationV2::Response::SharedPtr res) {
    res->success = false;
    res->header = req->cloud.header;
    res->globalmap_header = globalmap_header;
    if (!global_map) {
      res->success = false;
      res->rejection_reason = "no_global_map";
      return;
    }
    auto* fpfh_engine =
      dynamic_cast<GlobalLocalizationEngineFPFH_RANSAC*>(engine.get());
    if (!fpfh_engine) {
      res->success = false;
      res->rejection_reason = "active_engine_does_not_support_query_v2";
      return;
    }
    const bool has_normals = std::any_of(
      req->cloud.fields.begin(), req->cloud.fields.end(),
      [](const auto& field) { return field.name == "normal_x"; });
    if (!has_normals) {
      res->success = false;
      res->rejection_reason = "query_v2_requires_oriented_normals";
      return;
    }

    auto surface = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::fromROSMsg(req->cloud, *surface);
    pcl::fromROSMsg(req->cloud, *normals);
    const double deadline = req->matching_deadline_sec > 0.0
      ? req->matching_deadline_sec
      : 45.0;
    const auto results = fpfh_engine->query_v2(
      surface, normals, req->max_num_candidates, deadline);
    res->score_profile = results.score_profile;
    res->rejection_reason = results.rejection_reason;
    res->total_time_sec = results.total_time_sec;

    auto assign_pose = [](const Eigen::Isometry3f& transform, auto* pose) {
      const Eigen::Quaternionf quaternion(transform.linear());
      pose->position.x = transform.translation().x();
      pose->position.y = transform.translation().y();
      pose->position.z = transform.translation().z();
      pose->orientation.x = quaternion.x();
      pose->orientation.y = quaternion.y();
      pose->orientation.z = quaternion.z();
      pose->orientation.w = quaternion.w();
    };

    res->diagnostic_coarse_poses.resize(
      results.diagnostic_coarse.size());
    res->diagnostic_coarse_inliers.resize(
      results.diagnostic_coarse.size());
    res->diagnostic_coarse_errors.resize(
      results.diagnostic_coarse.size());
    for (std::size_t i = 0; i < results.diagnostic_coarse.size(); ++i) {
      assign_pose(
        results.diagnostic_coarse[i]->pose,
        &res->diagnostic_coarse_poses[i]);
      res->diagnostic_coarse_inliers[i] =
        results.diagnostic_coarse[i]->inlier_fraction;
      res->diagnostic_coarse_errors[i] =
        results.diagnostic_coarse[i]->error;
    }

    const std::size_t count = results.candidates.size();
    res->poses.resize(count);
    res->coarse_inliers.resize(count);
    res->coarse_errors.resize(count);
    res->fine_overlap_q2m.resize(count);
    res->fine_overlap_m2q.resize(count);
    res->fine_trimmed_rmse.resize(count);
    res->degeneracy_ratios.resize(count);
    res->top1_top2_margins.resize(count);
    res->refinement_status.resize(count);
    res->candidate_valid.resize(count);
    res->candidate_rejection_reasons.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto& candidate = results.candidates[i];
      assign_pose(candidate.pose, &res->poses[i]);
      res->coarse_inliers[i] = candidate.coarse_inlier;
      res->coarse_errors[i] = candidate.coarse_error;
      res->fine_overlap_q2m[i] = candidate.fine_overlap_q2m;
      res->fine_overlap_m2q[i] = candidate.fine_overlap_m2q;
      res->fine_trimmed_rmse[i] = candidate.fine_trimmed_rmse;
      res->degeneracy_ratios[i] = candidate.degeneracy_ratio;
      res->top1_top2_margins[i] = candidate.top1_top2_margin;
      res->refinement_status[i] = candidate.refinement_status;
      res->candidate_valid[i] = candidate.valid;
      res->candidate_rejection_reasons[i] =
        candidate.rejection_reason;
      res->success = res->success || candidate.valid;
    }
  }

private:
  double globalmap_downsample_resolution;
  double query_downsample_resolution;
  std::string active_fpfh_profile;
  GlobalLocalizationEngineFPFH_RANSACParams fpfh_params;

  rclcpp::ServiceBase::SharedPtr set_engine_service;
  rclcpp::ServiceBase::SharedPtr set_global_map_service;
  rclcpp::ServiceBase::SharedPtr load_global_map_cache_service;
  rclcpp::ServiceBase::SharedPtr query_service;
  rclcpp::ServiceBase::SharedPtr query_v2_service;

  std_msgs::msg::Header globalmap_header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
  MapFeatureCache::Ptr global_map_cache;
  std::unique_ptr<GlobalLocalizationEngine> engine;
};

}  // namespace hdl_global_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<hdl_global_localization::GlobalLocalizationNode>(options);
  rclcpp::spin(node);

  return 0;
}
