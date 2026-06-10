#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Scancontext.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <tuple>
#include <cmath>

struct KeyframePose
{
  int id;
  double x;
  double y;
  double z;
  double yaw;
  std::string pcd_file;
};

class ScanContextRelocalization : public rclcpp::Node
{
public:
  ScanContextRelocalization()
  : Node("scan_context_relocalization")
  {
    initialpose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_body",
      10,
      std::bind(
        &ScanContextRelocalization::cloudCallback,
        this,
        std::placeholders::_1));

    keyframe_dir_ = findLatestKeyframeDir();

    if (keyframe_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "No keyframes_xxx directory found!");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Using keyframe dir: %s",
      keyframe_dir_.c_str());

    loadMapDatabase();

    RCLCPP_INFO(
      get_logger(),
      "Scan Context relocalization node started, map keyframes=%zu",
      keyframes_.size());
  }

private:
  std::string findLatestKeyframeDir()
  {
    std::string base_dir =
      "/home/wheeltec/xz/rtabmap_nav2_stack/cloud_map";

    std::vector<std::string> dirs;

    if (!std::filesystem::exists(base_dir)) {
      return "";
    }

    for (const auto & entry : std::filesystem::directory_iterator(base_dir)) {
      if (!entry.is_directory()) {
        continue;
      }

      std::string name = entry.path().filename().string();

      if (name.rfind("keyframes_", 0) == 0) {
        dirs.push_back(entry.path().string());
      }
    }

    if (dirs.empty()) {
      return "";
    }

    std::sort(dirs.begin(), dirs.end());
    return dirs.back();
  }

  bool loadPoses()
  {
    std::string poses_path = keyframe_dir_ + "/poses.txt";

    std::ifstream file(poses_path);

    if (!file.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open poses.txt: %s",
        poses_path.c_str());
      return false;
    }

    std::string line;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::stringstream ss(line);

      KeyframePose pose;
      ss >> pose.id
         >> pose.x
         >> pose.y
         >> pose.z
         >> pose.yaw
         >> pose.pcd_file;

      if (ss.fail()) {
        continue;
      }

      keyframes_.push_back(pose);
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded poses: %zu",
      keyframes_.size());

    return !keyframes_.empty();
  }

  pcl::PointCloud<SCPointType>::Ptr convertXYZToSCPoint(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_xyz)
  {
    pcl::PointCloud<SCPointType>::Ptr cloud(
      new pcl::PointCloud<SCPointType>);

    cloud->points.reserve(cloud_xyz->points.size());

    for (const auto & p : cloud_xyz->points) {
      SCPointType q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;

      // 如果 SCPointType 是 PointXYZI，这里补一个默认 intensity
      q.intensity = 0.0;

      cloud->points.push_back(q);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
  }

  void loadMapDatabase()
  {
    if (!loadPoses()) {
      return;
    }

    int loaded_count = 0;

    for (const auto & kf : keyframes_) {
      std::string pcd_path = keyframe_dir_ + "/" + kf.pcd_file;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_xyz) != 0) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to load pcd: %s",
          pcd_path.c_str());
        continue;
      }

      if (cloud_xyz->empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Empty pcd: %s",
          pcd_path.c_str());
        continue;
      }

      auto cloud = convertXYZToSCPoint(cloud_xyz);

      sc_manager_.makeAndSaveScancontextAndKeys(*cloud);
      loaded_count++;

      RCLCPP_INFO(
        get_logger(),
        "Loaded keyframe id=%d, points=%zu",
        kf.id,
        cloud->points.size());
    }

    map_context_count_ = loaded_count;

    RCLCPP_INFO(
      get_logger(),
      "Map Scan Context database loaded: %d keyframes",
      map_context_count_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (map_context_count_ <= 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *this->get_clock(),
        3000,
        "Map database is empty");
      return;
    }

    pcl::PointCloud<SCPointType>::Ptr cloud(
      new pcl::PointCloud<SCPointType>);

    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Received empty cloud");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *this->get_clock(),
      3000,
      "Receive current cloud, points=%zu",
      cloud->points.size());

    sc_manager_.makeAndSaveScancontextAndKeys(*cloud);

    int matched_id = -1;
    float yaw_diff_rad = 0.0;

    std::tie(matched_id, yaw_diff_rad) =
      sc_manager_.detectLoopClosureID();

    if (matched_id < 0 || matched_id >= map_context_count_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *this->get_clock(),
        3000,
        "No valid Scan Context match");
      return;
    }

    const auto & matched_pose = keyframes_[matched_id];

    double final_yaw = matched_pose.yaw + yaw_diff_rad;

    RCLCPP_INFO(
      get_logger(),
      "MATCH keyframe_index=%d, node_id=%d, x=%.3f, y=%.3f, yaw=%.2f deg",
      matched_id,
      matched_pose.id,
      matched_pose.x,
      matched_pose.y,
      final_yaw * 180.0 / M_PI);

    publishInitialPose(
      matched_pose.x,
      matched_pose.y,
      matched_pose.z,
      final_yaw);
  }

  void publishInitialPose(
    double x,
    double y,
    double z,
    double yaw)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

    for (int i = 0; i < 36; i++) {
      msg.pose.covariance[i] = 0.0;
    }

    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.0685;

    initialpose_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "/initialpose published");
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  SCManager sc_manager_;

  std::string keyframe_dir_;
  std::vector<KeyframePose> keyframes_;
  int map_context_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ScanContextRelocalization>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
