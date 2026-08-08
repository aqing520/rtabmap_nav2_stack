#include <hdl_global_localization/cache/map_feature_cache.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

namespace hdl_global_localization {
namespace {

std::string bytes_to_hex(const unsigned char* digest, std::size_t size) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < size; ++i) {
    stream << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return stream.str();
}

std::string sha256_stream(std::istream& input) {
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (!context) {
    return {};
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  bool success = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1;
  std::array<char, 1024 * 1024> buffer{};
  while (success && input) {
    input.read(buffer.data(), buffer.size());
    const auto count = input.gcount();
    if (count > 0) {
      success = EVP_DigestUpdate(
        context, buffer.data(), static_cast<std::size_t>(count)) == 1;
    }
  }
  if (success) {
    success = EVP_DigestFinal_ex(context, digest.data(), &digest_size) == 1;
  }
  EVP_MD_CTX_free(context);
  return success ? bytes_to_hex(digest.data(), digest_size) : std::string();
}

bool finite_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  if (cloud.empty()) {
    return false;
  }
  for (const auto& point : cloud) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return false;
    }
  }
  return true;
}

bool finite_features(const pcl::PointCloud<pcl::FPFHSignature33>& features) {
  if (features.empty()) {
    return false;
  }
  for (const auto& feature : features) {
    for (float value : feature.histogram) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
  }
  return true;
}

bool artifact_matches(
  const std::filesystem::path& directory,
  const nlohmann::json& artifacts,
  const std::string& filename,
  std::string* error) {
  const auto path = directory / filename;
  if (!std::filesystem::is_regular_file(path)) {
    *error = "cache artifact missing: " + path.string();
    return false;
  }
  if (!artifacts.contains(filename) || !artifacts.at(filename).contains("sha256")) {
    *error = "manifest checksum missing for " + filename;
    return false;
  }
  const auto expected_size = artifacts.at(filename).value("size_bytes", std::uint64_t{});
  if (expected_size == 0 || std::filesystem::file_size(path) != expected_size) {
    *error = "cache artifact size mismatch: " + filename;
    return false;
  }
  const std::string actual = sha256_file(path.string());
  const std::string expected = artifacts.at(filename).at("sha256").get<std::string>();
  if (actual.empty() || actual != expected) {
    *error = "cache artifact checksum mismatch: " + filename;
    return false;
  }
  return true;
}

}  // namespace

std::string sha256_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return {};
  }
  return sha256_stream(input);
}

std::string sha256_text(const std::string& text) {
  std::istringstream input(text);
  return sha256_stream(input);
}

MapFeatureCache::Ptr MapFeatureCache::load(
  const std::string& cache_directory,
  const std::string& source_pcd_path,
  std::string* error) {
  namespace fs = std::filesystem;
  const auto started = std::chrono::steady_clock::now();
  const fs::path directory(cache_directory);
  const fs::path manifest_path = directory / "manifest.json";

  if (!fs::is_regular_file(manifest_path)) {
    *error = "cache manifest missing: " + manifest_path.string();
    return nullptr;
  }

  nlohmann::json manifest;
  try {
    std::ifstream stream(manifest_path);
    stream >> manifest;
  } catch (const std::exception& exception) {
    *error = std::string("failed to parse cache manifest: ") + exception.what();
    return nullptr;
  }

  if (manifest.value("schema_version", 0) != 1) {
    *error = "unsupported cache schema_version";
    return nullptr;
  }
  if (manifest.value("engine", std::string()) != "FPFH_RANSAC") {
    *error = "cache engine is not FPFH_RANSAC";
    return nullptr;
  }

  if (!manifest.contains("profile") || !manifest.at("profile").is_object()) {
    *error = "cache profile is missing or invalid";
    return nullptr;
  }
  const auto profile = manifest.at("profile");
  const std::string point_type =
    profile.value("point_type", std::string());
  if (point_type != "PointXYZ" && point_type != "PointXYZI") {
    *error = "unsupported cache point type for FPFH cache loader: " + point_type;
    return nullptr;
  }

  const auto cache_identity =
    manifest.value("cache_identity", nlohmann::json::object());
  if (cache_identity.value("schema_version", 0) != 1 ||
      cache_identity.value("cache_builder_version", std::string()) != "1") {
    *error = "unsupported cache identity";
    return nullptr;
  }
  if (cache_identity.value("pcl_major", -1) != PCL_MAJOR_VERSION ||
      cache_identity.value("pcl_minor", -1) != PCL_MINOR_VERSION) {
    *error = "cache PCL major/minor version does not match runtime";
    return nullptr;
  }
  nlohmann::json expected_profile_identity = profile;
  expected_profile_identity["schema_version"] =
    cache_identity.at("schema_version");
  expected_profile_identity["cache_builder_version"] =
    cache_identity.at("cache_builder_version");
  expected_profile_identity["pcl_major"] = cache_identity.at("pcl_major");
  expected_profile_identity["pcl_minor"] = cache_identity.at("pcl_minor");
  const std::string expected_profile_hash =
    sha256_text(expected_profile_identity.dump());
  const std::string manifest_profile_hash =
    manifest.value("profile_hash", std::string());
  if (expected_profile_hash.empty() ||
      expected_profile_hash != manifest_profile_hash) {
    *error = "cache profile hash does not match manifest profile";
    return nullptr;
  }

  const std::string expected_source_sha =
    manifest.value("source", nlohmann::json::object()).value("sha256", std::string());
  if (expected_source_sha.empty()) {
    *error = "cache source checksum is missing";
    return nullptr;
  }
  if (manifest.value("cache_key", std::string()) !=
      expected_source_sha + ":" + manifest_profile_hash) {
    *error = "cache key does not match source/profile hashes";
    return nullptr;
  }
  if (!source_pcd_path.empty()) {
    if (!fs::is_regular_file(source_pcd_path)) {
      *error = "source PCD is missing: " + source_pcd_path;
      return nullptr;
    }
    const auto expected_source_size =
      manifest.value("source", nlohmann::json::object())
        .value("size_bytes", std::uint64_t{});
    if (expected_source_size == 0 ||
        fs::file_size(source_pcd_path) != expected_source_size) {
      *error = "source PCD size does not match cache manifest";
      return nullptr;
    }
    const std::string actual_source_sha = sha256_file(source_pcd_path);
    if (actual_source_sha.empty() || actual_source_sha != expected_source_sha) {
      *error = "source PCD checksum does not match cache manifest";
      return nullptr;
    }
  }

  const auto artifacts = manifest.value("artifacts", nlohmann::json::object());
  for (const auto& filename : {"map_surface.pcd", "map_keypoints.pcd", "map_fpfh.pcd"}) {
    if (!artifact_matches(directory, artifacts, filename, error)) {
      return nullptr;
    }
  }

  auto cache = std::make_shared<MapFeatureCache>();
  cache->surface.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cache->keypoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cache->features.reset(new pcl::PointCloud<pcl::FPFHSignature33>);

  if (pcl::io::loadPCDFile((directory / "map_surface.pcd").string(), *cache->surface) < 0 ||
      pcl::io::loadPCDFile((directory / "map_keypoints.pcd").string(), *cache->keypoints) < 0 ||
      pcl::io::loadPCDFile((directory / "map_fpfh.pcd").string(), *cache->features) < 0) {
    *error = "failed to load one or more cache PCD artifacts";
    return nullptr;
  }

  if (!finite_cloud(*cache->surface) || !finite_cloud(*cache->keypoints) ||
      !finite_features(*cache->features)) {
    *error = "cache contains empty or non-finite point/feature data";
    return nullptr;
  }
  if (cache->keypoints->size() != cache->features->size()) {
    *error = "keypoint/FPFH count mismatch";
    return nullptr;
  }

  const auto counts = manifest.value("counts", nlohmann::json::object());
  if (counts.value("surface_points", std::uint64_t{}) != cache->surface->size() ||
      counts.value("keypoint_points", std::uint64_t{}) != cache->keypoints->size() ||
      counts.value("feature_points", std::uint64_t{}) != cache->features->size()) {
    *error = "manifest point counts do not match cache artifacts";
    return nullptr;
  }

  cache->cache_key = manifest.value("cache_key", directory.filename().string());
  cache->profile_name = manifest.value("profile_name", std::string());
  cache->profile_hash = manifest_profile_hash;
  cache->source_sha256 = expected_source_sha;
  cache->point_type = point_type;
  cache->artifact_load_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  return cache;
}

}  // namespace hdl_global_localization
