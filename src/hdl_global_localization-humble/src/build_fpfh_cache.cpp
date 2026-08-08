#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <ctime>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

#include <hdl_global_localization/cache/map_feature_cache.hpp>

namespace fs = std::filesystem;
using hdl_global_localization::sha256_file;
using hdl_global_localization::sha256_text;

namespace {

struct Arguments {
  std::string source_pcd;
  std::string prepared_map;
  std::string prepared_surface;
  std::string prepared_keypoints;
  std::string validate_cache;
  std::string cache_root;
  std::string profile_json;
  std::string profile_name = "legacy_fpfh_v1";
  bool force = false;
  bool print_identity = false;
};

void usage(const char* program) {
  std::cerr
    << "Usage: " << program << " --source-pcd FILE --prepared-map FILE "
    << "--cache-root DIR --profile-json FILE [--profile NAME] [--force]\n"
    << "       " << program << " --source-pcd FILE --prepared-surface FILE "
    << "--prepared-keypoints FILE --cache-root DIR --profile-json FILE "
    << "--profile garage_structural_v1 [--force]\n"
    << "       " << program << " --validate-cache DIR --source-pcd FILE\n"
    << "       " << program << " --print-identity\n";
}

bool parse_arguments(int argc, char** argv, Arguments* arguments) {
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    auto require_value = [&](std::string* output) {
      if (i + 1 >= argc) {
        return false;
      }
      *output = argv[++i];
      return true;
    };

    if (option == "--source-pcd") {
      if (!require_value(&arguments->source_pcd)) return false;
    } else if (option == "--prepared-map") {
      if (!require_value(&arguments->prepared_map)) return false;
    } else if (option == "--prepared-surface") {
      if (!require_value(&arguments->prepared_surface)) return false;
    } else if (option == "--prepared-keypoints") {
      if (!require_value(&arguments->prepared_keypoints)) return false;
    } else if (option == "--validate-cache") {
      if (!require_value(&arguments->validate_cache)) return false;
    } else if (option == "--cache-root") {
      if (!require_value(&arguments->cache_root)) return false;
    } else if (option == "--profile-json") {
      if (!require_value(&arguments->profile_json)) return false;
    } else if (option == "--profile") {
      if (!require_value(&arguments->profile_name)) return false;
    } else if (option == "--force") {
      arguments->force = true;
    } else if (option == "--print-identity") {
      arguments->print_identity = true;
    } else if (option == "-h" || option == "--help") {
      usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown option: " << option << "\n";
      return false;
    }
  }

  return arguments->print_identity ||
         (!arguments->validate_cache.empty() &&
         !arguments->source_pcd.empty()) ||
         (!arguments->source_pcd.empty() &&
         (!arguments->prepared_map.empty() ||
          (!arguments->prepared_surface.empty() &&
           !arguments->prepared_keypoints.empty())) &&
         !arguments->cache_root.empty() &&
         !arguments->profile_json.empty());
}

double elapsed(const std::chrono::steady_clock::time_point& started) {
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
}

bool valid_feature(const pcl::FPFHSignature33& feature) {
  for (float value : feature.histogram) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

void validate_features(
  const pcl::PointCloud<pcl::FPFHSignature33>& features) {
  if (features.empty()) {
    throw std::runtime_error("computed FPFH is empty");
  }
  for (const auto& feature : features) {
    if (!valid_feature(feature)) {
      throw std::runtime_error("computed FPFH contains non-finite values");
    }
  }
}

void validate_normals(const pcl::PointCloud<pcl::Normal>& normals) {
  if (normals.empty()) {
    throw std::runtime_error("surface normals are empty");
  }
  for (const auto& normal : normals) {
    const float norm = std::sqrt(
      normal.normal_x * normal.normal_x +
      normal.normal_y * normal.normal_y +
      normal.normal_z * normal.normal_z);
    if (!std::isfinite(normal.normal_x) ||
        !std::isfinite(normal.normal_y) ||
        !std::isfinite(normal.normal_z) ||
        norm <= 1.0e-6f) {
      throw std::runtime_error("surface normals contain non-finite or zero vectors");
    }
  }
}

std::string utc_timestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  Arguments arguments;
  if (!parse_arguments(argc, argv, &arguments)) {
    usage(argv[0]);
    return 2;
  }

  if (arguments.print_identity) {
    nlohmann::json identity = {
      {"schema_version", 1},
      {"cache_builder_version", "1"},
      {"pcl_major", PCL_MAJOR_VERSION},
      {"pcl_minor", PCL_MINOR_VERSION},
      {"pcl_version", PCL_VERSION_PRETTY}
    };
    std::cout << identity.dump() << "\n";
    return 0;
  }

  try {
    if (!arguments.validate_cache.empty()) {
      std::string load_error;
      const auto existing = hdl_global_localization::MapFeatureCache::load(
        arguments.validate_cache, arguments.source_pcd, &load_error);
      if (!existing) {
        throw std::runtime_error("cache validation failed: " + load_error);
      }
      std::cout << "CACHE_HIT: true\n";
      std::cout << "CACHE_DIR: " << fs::absolute(arguments.validate_cache).string() << "\n";
      std::cout << "CACHE_KEY: " << existing->cache_key << "\n";
      std::cout << "SURFACE_POINTS: " << existing->surface->size() << "\n";
      std::cout << "FEATURE_POINTS: " << existing->features->size() << "\n";
      std::cout << "CACHE_LOAD_SEC: " << existing->artifact_load_sec << "\n";
      return 0;
    }

    if (!fs::is_regular_file(arguments.source_pcd)) {
      throw std::runtime_error("source PCD does not exist");
    }
    nlohmann::json root_config;
    {
      std::ifstream stream(arguments.profile_json);
      if (!stream) {
        throw std::runtime_error("failed to open profile JSON");
      }
      stream >> root_config;
    }
    if (!root_config.contains("profiles") ||
        !root_config.at("profiles").contains(arguments.profile_name)) {
      throw std::runtime_error("profile not found: " + arguments.profile_name);
    }
    const nlohmann::json profile =
      root_config.at("profiles").at(arguments.profile_name);
    const bool legacy_profile = arguments.profile_name == "legacy_fpfh_v1";
    if (legacy_profile) {
      if (!fs::is_regular_file(arguments.prepared_map)) {
        throw std::runtime_error("prepared map does not exist");
      }
    } else {
      if (!fs::is_regular_file(arguments.prepared_surface)) {
        throw std::runtime_error("prepared structural surface does not exist");
      }
      if (!fs::is_regular_file(arguments.prepared_keypoints)) {
        throw std::runtime_error("prepared structural keypoints do not exist");
      }
    }

    const std::string source_sha = sha256_file(arguments.source_pcd);
    if (source_sha.empty()) {
      throw std::runtime_error("failed to hash source PCD");
    }

    nlohmann::json profile_identity = profile;
    profile_identity["schema_version"] = 1;
    profile_identity["cache_builder_version"] = "1";
    profile_identity["pcl_major"] = PCL_MAJOR_VERSION;
    profile_identity["pcl_minor"] = PCL_MINOR_VERSION;
    const std::string profile_hash = sha256_text(profile_identity.dump());
    const std::string cache_key = source_sha + ":" + profile_hash;
    const fs::path cache_directory =
      fs::path(arguments.cache_root) / source_sha / profile_hash;
    const fs::path manifest_path = cache_directory / "manifest.json";

    if (!arguments.force && fs::is_regular_file(manifest_path)) {
      std::string load_error;
      const auto existing = hdl_global_localization::MapFeatureCache::load(
        cache_directory.string(), arguments.source_pcd, &load_error);
      if (existing) {
        std::cout << "CACHE_HIT: true\n";
        std::cout << "CACHE_DIR: " << cache_directory.string() << "\n";
        std::cout << "CACHE_KEY: " << existing->cache_key << "\n";
        return 0;
      }
      std::cerr << "[WARN] Existing cache is invalid and will be rebuilt: "
                << load_error << "\n";
    }

    const auto build_started = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(
      new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(
      new pcl::PointCloud<pcl::FPFHSignature33>);
    double prepared_load_sec = 0.0;
    double downsample_sec = 0.0;
    double normal_sec = 0.0;
    double fpfh_sec = 0.0;
    std::uint64_t prepared_points = 0;

    if (legacy_profile) {
      const auto load_started = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZ>::Ptr prepared(
        new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile(arguments.prepared_map, *prepared) < 0 ||
          prepared->empty()) {
        throw std::runtime_error("failed to load prepared map");
      }
      prepared_load_sec = elapsed(load_started);
      prepared_points = prepared->size();

      const auto downsample_started = std::chrono::steady_clock::now();
      pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_grid;
      const float map_voxel =
        profile.value("globalmap_downsample_resolution", 0.2);
      voxel_grid.setLeafSize(map_voxel, map_voxel, map_voxel);
      voxel_grid.setInputCloud(prepared);
      voxel_grid.filter(*surface);
      downsample_sec = elapsed(downsample_started);
      if (surface->empty()) {
        throw std::runtime_error(
          "prepared map became empty after downsampling");
      }

      const auto normal_started = std::chrono::steady_clock::now();
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
      normal_estimation.setRadiusSearch(profile.value("normal_radius", 0.5));
      normal_estimation.setInputCloud(surface);
      normal_estimation.compute(*normals);
      normal_sec = elapsed(normal_started);

      const auto feature_started = std::chrono::steady_clock::now();
      pcl::FPFHEstimationOMP<
        pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setRadiusSearch(profile.value("fpfh_radius", 1.5));
      fpfh.setInputCloud(surface);
      fpfh.setInputNormals(normals);
      fpfh.compute(*features);
      fpfh_sec = elapsed(feature_started);
      keypoints = surface;
    } else {
      const auto load_started = std::chrono::steady_clock::now();
      if (pcl::io::loadPCDFile(arguments.prepared_surface, *surface) < 0 ||
          surface->empty()) {
        throw std::runtime_error("failed to load structural surface");
      }
      if (pcl::io::loadPCDFile(arguments.prepared_keypoints, *keypoints) < 0 ||
          keypoints->empty()) {
        throw std::runtime_error("failed to load structural keypoints");
      }
      if (pcl::io::loadPCDFile(arguments.prepared_surface, *normals) < 0 ||
          normals->empty()) {
        throw std::runtime_error("failed to load structural surface normals");
      }
      prepared_load_sec = elapsed(load_started);
      prepared_points = surface->size();

      validate_normals(*normals);
      if (surface->size() != normals->size()) {
        throw std::runtime_error(
          "structural surface and normals size mismatch");
      }
      if (keypoints->empty()) {
        throw std::runtime_error("structural keypoints are empty");
      }

      const auto feature_started = std::chrono::steady_clock::now();
      pcl::FPFHEstimationOMP<
        pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setRadiusSearch(profile.value("fpfh_radius", 1.5));
      fpfh.setInputCloud(keypoints);
      fpfh.setSearchSurface(surface);
      fpfh.setInputNormals(normals);
      fpfh.compute(*features);
      fpfh_sec = elapsed(feature_started);
    }

    if (features->size() != keypoints->size()) {
      throw std::runtime_error("keypoint/FPFH count mismatch after computation");
    }
    validate_features(*features);

    fs::create_directories(cache_directory.parent_path());
    const fs::path temporary_directory =
      cache_directory.parent_path() /
      (profile_hash + ".tmp." + std::to_string(::getpid()));
    std::error_code error_code;
    fs::remove_all(temporary_directory, error_code);
    fs::create_directories(temporary_directory);

    const auto io_started = std::chrono::steady_clock::now();
    const fs::path surface_path = temporary_directory / "map_surface.pcd";
    const fs::path keypoints_path = temporary_directory / "map_keypoints.pcd";
    const fs::path features_path = temporary_directory / "map_fpfh.pcd";
    if (legacy_profile) {
      if (pcl::io::savePCDFileBinaryCompressed(surface_path.string(), *surface) < 0 ||
          pcl::io::savePCDFileBinaryCompressed(keypoints_path.string(), *keypoints) < 0) {
        throw std::runtime_error("failed to write cache point artifacts");
      }
    } else {
      fs::copy_file(
        arguments.prepared_surface, surface_path,
        fs::copy_options::overwrite_existing);
      fs::copy_file(
        arguments.prepared_keypoints, keypoints_path,
        fs::copy_options::overwrite_existing);
    }
    if (pcl::io::savePCDFileBinaryCompressed(
          features_path.string(), *features) < 0) {
      throw std::runtime_error("failed to write cache FPFH artifact");
    }
    const double artifact_write_sec = elapsed(io_started);

    nlohmann::json manifest;
    manifest["schema_version"] = 1;
    manifest["engine"] = "FPFH_RANSAC";
    manifest["cache_key"] = cache_key;
    manifest["profile_name"] = arguments.profile_name;
    manifest["profile_hash"] = profile_hash;
    manifest["cache_identity"] = {
      {"schema_version", 1},
      {"cache_builder_version", "1"},
      {"pcl_major", PCL_MAJOR_VERSION},
      {"pcl_minor", PCL_MINOR_VERSION}
    };
    manifest["source"] = {
      {"path", fs::absolute(arguments.source_pcd).string()},
      {"size_bytes", fs::file_size(arguments.source_pcd)},
      {"sha256", source_sha}
    };
    manifest["profile"] = profile;
    manifest["build"] = {
      {"pcl_version", PCL_VERSION_PRETTY},
      {"cache_builder_version", "1"},
      {"created_at_utc", utc_timestamp()},
      {"git_commit", std::getenv("HDL_CACHE_GIT_COMMIT") ?
        std::getenv("HDL_CACHE_GIT_COMMIT") : "unknown"}
    };
    manifest["counts"] = {
      {"prepared_points", prepared_points},
      {"surface_points", surface->size()},
      {"keypoint_points", keypoints->size()},
      {"feature_points", features->size()}
    };
    manifest["timing_sec"] = {
      {"prepared_map_load", prepared_load_sec},
      {"map_downsample", downsample_sec},
      {"normal_estimation", normal_sec},
      {"fpfh", fpfh_sec},
      {"artifact_write", artifact_write_sec},
      {"total", elapsed(build_started)}
    };
    manifest["artifacts"] = {
      {"map_surface.pcd", {
        {"size_bytes", fs::file_size(surface_path)},
        {"sha256", sha256_file(surface_path.string())}
      }},
      {"map_keypoints.pcd", {
        {"size_bytes", fs::file_size(keypoints_path)},
        {"sha256", sha256_file(keypoints_path.string())}
      }},
      {"map_fpfh.pcd", {
        {"size_bytes", fs::file_size(features_path)},
        {"sha256", sha256_file(features_path.string())}
      }}
    };
    {
      std::ofstream stream(temporary_directory / "manifest.json");
      stream << std::setw(2) << manifest << "\n";
    }

    std::string validation_error;
    const auto validated = hdl_global_localization::MapFeatureCache::load(
      temporary_directory.string(), arguments.source_pcd, &validation_error);
    if (!validated) {
      throw std::runtime_error(
        "new cache failed validation: " + validation_error);
    }

    const fs::path backup_directory =
      cache_directory.parent_path() /
      (profile_hash + ".backup." + std::to_string(::getpid()));
    fs::remove_all(backup_directory, error_code);
    bool moved_existing = false;
    try {
      if (fs::exists(cache_directory)) {
        fs::rename(cache_directory, backup_directory);
        moved_existing = true;
      }
      fs::rename(temporary_directory, cache_directory);
      if (moved_existing) {
        fs::remove_all(backup_directory, error_code);
      }
    } catch (...) {
      if (moved_existing && !fs::exists(cache_directory) &&
          fs::exists(backup_directory)) {
        std::error_code rollback_error;
        fs::rename(backup_directory, cache_directory, rollback_error);
      }
      throw;
    }

    std::cout << "CACHE_HIT: false\n";
    std::cout << "CACHE_DIR: " << cache_directory.string() << "\n";
    std::cout << "CACHE_KEY: " << cache_key << "\n";
    std::cout << "SURFACE_POINTS: " << surface->size() << "\n";
    std::cout << "KEYPOINT_POINTS: " << keypoints->size() << "\n";
    std::cout << "FEATURE_POINTS: " << features->size() << "\n";
    std::cout << "NORMAL_SEC: " << normal_sec << "\n";
    std::cout << "FPFH_SEC: " << fpfh_sec << "\n";
    std::cout << "TOTAL_SEC: " << elapsed(build_started) << "\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "[ERROR] " << exception.what() << "\n";
    return 1;
  }
}
