#ifndef HDL_GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP
#define HDL_GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP

#include <limits>
#include <string>
#include <vector>

#include <hdl_global_localization/engines/global_localization_engine.hpp>
#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>
#include <hdl_global_localization/cache/map_feature_cache.hpp>

namespace hdl_global_localization {

struct FineScoringParams {
  bool enabled = false;
  bool se2_refinement_enabled = false;
  bool score_profile_calibrated = false;
  std::string score_profile = "unconfigured";

  int internal_candidate_pool = 100;
  int diagnostic_candidate_count = 20;
  int refinement_top_k = 5;
  int target_max_points = 100000;
  double query_keypoint_voxel = 0.2;
  double cluster_translation_m = 1.0;
  double cluster_yaw_rad = 0.2617993877991494;
  double local_map_margin_m = 5.0;
  double fine_correspondence_distance = 0.25;
  double fine_trim_fraction = 0.8;
  double se2_icp_coarse_correspondence = 0.8;
  int se2_icp_coarse_iterations = 20;
  double se2_icp_fine_correspondence = 0.25;
  int se2_icp_fine_iterations = 30;

  double min_fine_overlap_q2m = 0.0;
  double min_fine_overlap_m2q = 0.0;
  double max_fine_trimmed_rmse = std::numeric_limits<double>::infinity();
  double min_degeneracy_ratio = 0.0;
  double min_top1_top2_margin = 0.0;
};

struct GlobalLocalizationEngineFPFH_RANSACParams {
  RansacPoseEstimationParams ransac_params;

  double normal_estimation_radius = 0.5;
  double search_radius = 1.5;
  FineScoringParams fine;
};

struct DetailedGlobalLocalizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  double coarse_inlier = 0.0;
  double coarse_error = std::numeric_limits<double>::infinity();
  double fine_overlap_q2m = 0.0;
  double fine_overlap_m2q = 0.0;
  double fine_trimmed_rmse = std::numeric_limits<double>::infinity();
  double degeneracy_ratio = 0.0;
  double top1_top2_margin = 0.0;
  double quality = -std::numeric_limits<double>::infinity();
  std::string refinement_status;
  bool valid = false;
  std::string rejection_reason;
};

struct DetailedGlobalLocalizationResults {
  std::string score_profile;
  std::string rejection_reason;
  double total_time_sec = 0.0;
  std::vector<GlobalLocalizationResult::Ptr> diagnostic_coarse;
  std::vector<DetailedGlobalLocalizationResult> candidates;
};

class GlobalLocalizationEngineFPFH_RANSAC : public GlobalLocalizationEngine {
public:
  GlobalLocalizationEngineFPFH_RANSAC(const GlobalLocalizationEngineFPFH_RANSACParams& params = GlobalLocalizationEngineFPFH_RANSACParams());
  virtual ~GlobalLocalizationEngineFPFH_RANSAC() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;
  void set_global_map_cache(const MapFeatureCache::Ptr& cache);
  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;
  DetailedGlobalLocalizationResults query_v2(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr surface,
    pcl::PointCloud<pcl::Normal>::ConstPtr surface_normals,
    int max_num_candidates,
    double matching_deadline_sec);

protected:
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr extract_fpfh(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr surface,
    pcl::PointCloud<pcl::Normal>::ConstPtr surface_normals);
  GlobalLocalizationResults query_coarse(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr keypoints,
    pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr features,
    int candidate_pool);

protected:
  const GlobalLocalizationEngineFPFH_RANSACParams params;

  std::unique_ptr<RansacPoseEstimation<pcl::FPFHSignature33>> ransac;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map_keypoints;
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr global_map_features;
};

}  // namespace hdl_global_localization

#endif
