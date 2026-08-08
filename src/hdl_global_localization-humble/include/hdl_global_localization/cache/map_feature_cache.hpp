#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hdl_global_localization {

struct MapFeatureCache {
  using Ptr = std::shared_ptr<MapFeatureCache>;

  std::string cache_key;
  std::string profile_name;
  std::string profile_hash;
  std::string source_sha256;
  std::string point_type;

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface;
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features;

  double artifact_load_sec = 0.0;

  static Ptr load(
    const std::string& cache_directory,
    const std::string& source_pcd_path,
    std::string* error);
};

std::string sha256_file(const std::string& path);
std::string sha256_text(const std::string& text);

}  // namespace hdl_global_localization
