#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace visual_initial_pose
{

using namespace std::chrono_literals;

class VisualInitialPoseNode : public rclcpp::Node
{
public:
  VisualInitialPoseNode()
  : Node("visual_initial_pose"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_and_read_parameters();

    rclcpp::QoS latched_qos(1);
    latched_qos.reliable().transient_local();
    visual_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      visual_pose_topic_, latched_qos);
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, latched_qos);
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&VisualInitialPoseNode::initialpose_callback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);
    success_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/visual_initial_pose/success", latched_qos);
    candidate_id_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/visual_initial_pose/candidate_id", latched_qos);
    candidate_score_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/visual_initial_pose/candidate_score", latched_qos);

    navigation_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      navigation_manager_service_);

    started_at_ = std::chrono::steady_clock::now();
    last_process_at_ = started_at_ -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(process_interval_sec_));

    if (initialize_database()) {
      rgbd_sub_ = create_subscription<rtabmap_msgs::msg::RGBDImage>(
        rgbd_topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&VisualInitialPoseNode::rgbd_callback, this, std::placeholders::_1));
      state_ = State::SEARCHING;
      publish_status("searching", "database loaded; waiting for RGB-D data");
    } else {
      state_ = State::FAILED;
      publish_status("failed", "database initialization failed");
    }

    timer_ = create_wall_timer(200ms, std::bind(&VisualInitialPoseNode::timer_callback, this));
  }

  ~VisualInitialPoseNode() override
  {
    if (database_initialized_) {
      try {
        rtabmap_.close(false);
      } catch (...) {
        // Destructors must not throw while ROS is shutting down.
      }
    }
  }

private:
  enum class State
  {
    INITIALIZING,
    SEARCHING,
    WAITING_TF_CONFIRMATION,
    ACTIVATING_NAVIGATION,
    COMPLETE,
    FAILED,
  };

  static std::string expand_user(const std::string & path)
  {
    if (path.empty() || path[0] != '~') {
      return path;
    }
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      return path;
    }
    if (path.size() == 1) {
      return std::string(home);
    }
    if (path[1] == '/') {
      return std::string(home) + path.substr(1);
    }
    return path;
  }

  static float statistic_value(
    const rtabmap::Statistics & statistics,
    const std::string & key,
    float default_value = 0.0f)
  {
    const auto & values = statistics.data();
    const auto iter = values.find(key);
    return iter == values.end() ? default_value : iter->second;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void declare_and_read_parameters()
  {
    database_path_ = expand_user(declare_parameter<std::string>("database_path", ""));
    rgbd_topic_ = declare_parameter<std::string>("rgbd_topic", "/rgbd_image");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    visual_pose_topic_ = declare_parameter<std::string>(
      "visual_pose_topic", "/visual_initialpose");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/visual_initial_pose/status");

    process_interval_sec_ = std::max(
      0.1, declare_parameter<double>("process_interval_sec", 1.0));
    sensor_wait_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("sensor_wait_timeout_sec", 30.0));
    localization_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("localization_timeout_sec", 30.0));
    continue_search_after_timeout_ = declare_parameter<bool>(
      "continue_search_after_timeout", true);
    tf_lookup_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("tf_lookup_timeout_sec", 0.2));

    min_hypothesis_ = std::max(
      0.0, declare_parameter<double>("min_hypothesis", 0.11));
    min_visual_inliers_ = std::max(
      0, static_cast<int>(declare_parameter<int>("min_visual_inliers", 15)));
    min_visual_inliers_ratio_ = std::max(
      0.0, declare_parameter<double>("min_visual_inliers_ratio", 0.0));
    min_best_second_ratio_ = std::max(
      1.0, declare_parameter<double>("min_best_second_ratio", 1.0));
    max_optimization_error_ratio_ = std::max(
      0.0, declare_parameter<double>("max_optimization_error_ratio", 3.0));
    max_linear_variance_ = std::max(
      0.0, declare_parameter<double>("max_linear_variance", 1.0));
    max_yaw_variance_ = std::max(
      0.0, declare_parameter<double>("max_yaw_variance", 1.0));
    required_confirmations_ = std::max(
      1, static_cast<int>(declare_parameter<int>("required_confirmations", 1)));

    detector_strategy_ = declare_parameter<int>("detector_strategy", -1);
    feature_type_ = declare_parameter<int>("feature_type", -1);
    max_features_ = std::max(
      0, static_cast<int>(declare_parameter<int>("max_features", 800)));
    estimation_type_ = declare_parameter<int>("estimation_type", -1);
    flatten_to_2d_ = declare_parameter<bool>("flatten_to_2d", true);

    publish_initialpose_ = declare_parameter<bool>("publish_initialpose", true);
    initialpose_publish_repetitions_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("initialpose_publish_repetitions", 3)));
    initialpose_publish_interval_sec_ = std::max(
      0.05, declare_parameter<double>("initialpose_publish_interval_sec", 0.2));
    fallback_linear_variance_ = std::max(
      0.0, declare_parameter<double>("fallback_linear_variance", 0.25));
    fallback_yaw_variance_ = std::max(
      0.0, declare_parameter<double>("fallback_yaw_variance", 0.068));
    allow_last_pose_fallback_ = declare_parameter<bool>("allow_last_pose_fallback", false);

    activate_nav2_on_success_ = declare_parameter<bool>("activate_nav2_on_success", false);
    tf_confirmation_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("tf_confirmation_timeout_sec", 10.0));
    tf_settle_delay_sec_ = std::max(
      0.0, declare_parameter<double>("tf_settle_delay_sec", 0.5));
    tf_confirmation_linear_tolerance_ = std::max(
      0.0, declare_parameter<double>("tf_confirmation_linear_tolerance", 0.5));
    tf_confirmation_yaw_tolerance_ = std::max(
      0.0, declare_parameter<double>("tf_confirmation_yaw_tolerance", 0.5));
    lifecycle_service_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("lifecycle_service_timeout_sec", 30.0));
    navigation_manager_service_ = declare_parameter<std::string>(
      "navigation_manager_service", "/lifecycle_manager_navigation/manage_nodes");

    if (activate_nav2_on_success_ && !publish_initialpose_) {
      RCLCPP_WARN(
        get_logger(),
        "activate_nav2_on_success=true requires publish_initialpose=true; disabling activation.");
      activate_nav2_on_success_ = false;
    }
  }

  bool initialize_database()
  {
    if (database_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "database_path is empty.");
      return false;
    }
    if (!std::filesystem::exists(database_path_)) {
      RCLCPP_ERROR(get_logger(), "Database does not exist: %s", database_path_.c_str());
      return false;
    }

    rtabmap::ParametersMap parameters;
    parameters[rtabmap::Parameters::kMemIncrementalMemory()] = "false";
    parameters[rtabmap::Parameters::kMemLocalizationReadOnly()] = "true";
    parameters[rtabmap::Parameters::kMemLocalizationDataSaved()] = "false";
    parameters[rtabmap::Parameters::kMemInitWMWithAllNodes()] = "true";
    parameters[rtabmap::Parameters::kMemLoadVisualLocalFeaturesOnInit()] = "true";
    parameters[rtabmap::Parameters::kRtabmapPublishStats()] = "true";
    parameters[rtabmap::Parameters::kRtabmapPublishPdf()] = "true";
    parameters[rtabmap::Parameters::kRtabmapPublishLikelihood()] = "true";
    parameters[rtabmap::Parameters::kRtabmapLoopThr()] = std::to_string(min_hypothesis_);
    parameters[rtabmap::Parameters::kRegStrategy()] = "0";
    parameters[rtabmap::Parameters::kVisMinInliers()] = std::to_string(min_visual_inliers_);
    parameters[rtabmap::Parameters::kVisMaxFeatures()] = std::to_string(max_features_);
    parameters[rtabmap::Parameters::kRGBDLinearUpdate()] = "0";
    parameters[rtabmap::Parameters::kRGBDAngularUpdate()] = "0";
    parameters[rtabmap::Parameters::kRGBDProximityBySpace()] = "false";
    parameters[rtabmap::Parameters::kRGBDStartAtOrigin()] = "false";
    parameters[rtabmap::Parameters::kRtabmapMemoryThr()] = "0";

    if (detector_strategy_ >= 0) {
      parameters[rtabmap::Parameters::kKpDetectorStrategy()] =
        std::to_string(detector_strategy_);
    }
    if (feature_type_ >= 0) {
      parameters[rtabmap::Parameters::kVisFeatureType()] = std::to_string(feature_type_);
    }
    if (estimation_type_ >= 0) {
      parameters[rtabmap::Parameters::kVisEstimationType()] =
        std::to_string(estimation_type_);
    }

    try {
      RCLCPP_INFO(
        get_logger(), "Loading RTAB-Map database read-only: %s", database_path_.c_str());
      rtabmap_.init(parameters, database_path_, true);
      database_initialized_ = true;
      database_last_pose_ = rtabmap_.getLastLocalizationPose().clone();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "RTAB-Map database initialization failed: %s", error.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "RTAB-Map database initialization failed with unknown error.");
      return false;
    }

    if (rtabmap_.getWMSize() <= 0) {
      RCLCPP_ERROR(get_logger(), "Database contains no usable localization nodes.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Visual database ready: WM=%d, threshold=%.3f, min_inliers=%d, "
      "detector=%s, feature=%s",
      rtabmap_.getWMSize(), min_hypothesis_, min_visual_inliers_,
      detector_strategy_ < 0 ? "database" : std::to_string(detector_strategy_).c_str(),
      feature_type_ < 0 ? "database" : std::to_string(feature_type_).c_str());
    return true;
  }

  void rgbd_callback(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr msg)
  {
    if (state_ != State::SEARCHING || !database_initialized_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_process_at_).count() < process_interval_sec_) {
      return;
    }
    last_process_at_ = now;
    if (!first_frame_received_) {
      first_frame_received_ = true;
      first_frame_at_ = now;
      RCLCPP_INFO(get_logger(), "First RGB-D frame received; starting visual localization.");
    }

    rtabmap::SensorData data;
    try {
      data = rtabmap_conversions::rgbdImageFromROS(msg);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to convert RGB-D message: %s", error.what());
      return;
    }

    if (data.imageRaw().empty()) {
      RCLCPP_WARN(get_logger(), "Received RGB-D message without a usable RGB image.");
      return;
    }

    const std::string sensor_frame = !msg->header.frame_id.empty() ?
      msg->header.frame_id : msg->rgb.header.frame_id;
    if (sensor_frame.empty()) {
      RCLCPP_WARN(get_logger(), "RGB-D message has no sensor frame_id.");
      return;
    }

    const rtabmap::Transform base_to_sensor = rtabmap_conversions::getTransform(
      base_frame_, sensor_frame, rclcpp::Time(msg->header.stamp), tf_buffer_,
      tf_lookup_timeout_sec_);
    if (base_to_sensor.isNull()) {
      RCLCPP_WARN(
        get_logger(), "Cannot get TF %s -> %s for visual localization.",
        base_frame_.c_str(), sensor_frame.c_str());
      return;
    }

    if (!data.cameraModels().empty()) {
      auto models = data.cameraModels();
      for (auto & model : models) {
        model.setLocalTransform(base_to_sensor);
      }
      data.setCameraModels(models);
    } else if (!data.stereoCameraModels().empty()) {
      auto models = data.stereoCameraModels();
      for (auto & model : models) {
        model.setLocalTransform(base_to_sensor);
      }
      data.setStereoCameraModels(models);
    } else {
      RCLCPP_WARN(get_logger(), "RGB-D frame has no valid camera calibration model.");
      return;
    }

    data.setId(0);
    ++attempt_count_;
    try {
      const cv::Mat odom_covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0001;
      rtabmap_.process(data, rtabmap::Transform::getIdentity(), odom_covariance);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "RTAB-Map visual localization failed: %s", error.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "RTAB-Map visual localization failed with unknown error.");
      return;
    }

    evaluate_result(msg->header.stamp);
  }

  void initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (
      have_internal_initialpose_stamp_ &&
      msg->header.stamp.sec == last_internal_initialpose_stamp_.sec &&
      msg->header.stamp.nanosec == last_internal_initialpose_stamp_.nanosec)
    {
      return;
    }

    if (
      state_ == State::ACTIVATING_NAVIGATION ||
      state_ == State::COMPLETE)
    {
      RCLCPP_INFO(
        get_logger(), "Ignoring external %s because Nav2 activation is already in progress or complete.",
        initialpose_topic_.c_str());
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring external %s in frame \"%s\"; expected \"%s\".",
        initialpose_topic_.c_str(), msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    accepted_pose_msg_ = *msg;
    accepted_pose_msg_.header.frame_id = map_frame_;
    accepted_pose_ =
      rtabmap_conversions::transformFromPoseMsg(accepted_pose_msg_.pose.pose);
    if (accepted_pose_.isNull()) {
      RCLCPP_WARN(get_logger(), "Ignoring external %s with an invalid pose.", initialpose_topic_.c_str());
      return;
    }

    if (flatten_to_2d_) {
      float x, y, z, roll, pitch, yaw;
      accepted_pose_.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
      accepted_pose_ = rtabmap::Transform(x, y, 0.0f, 0.0f, 0.0f, yaw);
      rtabmap_conversions::transformToPoseMsg(
        accepted_pose_, accepted_pose_msg_.pose.pose);
    }

    initialpose_remaining_ = 0;
    lifecycle_request_in_flight_ = false;
    pose_published_at_ = std::chrono::steady_clock::now();
    tf_confirmation_started_at_ = pose_published_at_;

    visual_pose_pub_->publish(accepted_pose_msg_);
    std_msgs::msg::Bool success_msg;
    success_msg.data = true;
    success_pub_->publish(success_msg);

    publish_status(
      "manual",
      "received external /initialpose; waiting for map to base TF confirmation");
    RCLCPP_INFO(
      get_logger(), "External initial pose accepted: %s; waiting for TF confirmation.",
      accepted_pose_.prettyPrint().c_str());

    state_ = activate_nav2_on_success_ ?
      State::WAITING_TF_CONFIRMATION : State::COMPLETE;
  }

  void evaluate_result(const builtin_interfaces::msg::Time & stamp)
  {
    const auto & statistics = rtabmap_.getStatistics();
    const int loop_id = statistics.loopClosureId();
    const int candidate_id = rtabmap_.getHighestHypothesisId();
    const double score = rtabmap_.getHighestHypothesisValue();
    const int visual_inliers = static_cast<int>(statistic_value(
      statistics, rtabmap::Statistics::kLoopVisual_inliers()));
    const double visual_inliers_ratio = statistic_value(
      statistics, rtabmap::Statistics::kLoopVisual_inliers_ratio());
    const bool rejected = statistic_value(
      statistics, rtabmap::Statistics::kLoopRejectedHypothesis()) > 0.5f;
    const double optimization_error_ratio = statistic_value(
      statistics, rtabmap::Statistics::kLoopOptimization_max_error_ratio());
    const double best_second_ratio = compute_best_second_ratio(statistics.posterior());

    std_msgs::msg::Int32 id_msg;
    id_msg.data = candidate_id;
    candidate_id_pub_->publish(id_msg);
    std_msgs::msg::Float32 score_msg;
    score_msg.data = static_cast<float>(score);
    candidate_score_pub_->publish(score_msg);

    RCLCPP_INFO(
      get_logger(),
      "Visual attempt %d: candidate=%d score=%.3f loop=%d inliers=%d "
      "inlier_ratio=%.3f best/second=%.3f rejected=%s",
      attempt_count_, candidate_id, score, loop_id, visual_inliers,
      visual_inliers_ratio, best_second_ratio, rejected ? "true" : "false");

    std::vector<std::string> failures;
    if (loop_id <= 0) {
      failures.emplace_back("RTAB-Map did not accept a visual loop closure");
    }
    if (score < min_hypothesis_) {
      failures.emplace_back("hypothesis below threshold");
    }
    if (visual_inliers < min_visual_inliers_) {
      failures.emplace_back("not enough visual inliers");
    }
    if (visual_inliers_ratio < min_visual_inliers_ratio_) {
      failures.emplace_back("visual inlier ratio below threshold");
    }
    if (min_best_second_ratio_ > 1.0 && best_second_ratio < min_best_second_ratio_) {
      failures.emplace_back("best/second candidate ratio below threshold");
    }
    if (rejected) {
      failures.emplace_back("RTAB-Map rejected the hypothesis");
    }
    if (
      max_optimization_error_ratio_ > 0.0 &&
      optimization_error_ratio > max_optimization_error_ratio_)
    {
      failures.emplace_back("optimization error ratio above threshold");
    }

    std::string covariance_reason;
    if (!localization_covariance_valid(statistics.localizationCovariance(), covariance_reason)) {
      failures.emplace_back(covariance_reason);
    }

    if (!failures.empty()) {
      std::ostringstream detail;
      detail << "candidate=" << candidate_id << " score=" << score << ": ";
      for (std::size_t i = 0; i < failures.size(); ++i) {
        if (i != 0) {
          detail << "; ";
        }
        detail << failures[i];
      }
      publish_status("searching", detail.str());
      confirmation_count_ = 0;
      confirmed_node_id_ = 0;
      return;
    }

    if (confirmed_node_id_ == loop_id) {
      ++confirmation_count_;
    } else {
      confirmed_node_id_ = loop_id;
      confirmation_count_ = 1;
    }
    if (confirmation_count_ < required_confirmations_) {
      publish_status(
        "confirming",
        "accepted node " + std::to_string(loop_id) + " confirmation " +
        std::to_string(confirmation_count_) + "/" +
        std::to_string(required_confirmations_));
      return;
    }

    rtabmap::Transform pose = rtabmap_.getLastLocalizationPose().clone();
    if (pose.isNull()) {
      publish_status("searching", "accepted loop closure returned a null localization pose");
      return;
    }
    if (flatten_to_2d_) {
      float x, y, z, roll, pitch, yaw;
      pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
      pose = rtabmap::Transform(x, y, 0.0f, 0.0f, 0.0f, yaw);
    }

    auto pose_msg = make_pose_message(pose, stamp, statistics.localizationCovariance(), false);
    publish_localization(pose_msg, loop_id, score, false);
  }

  double compute_best_second_ratio(const std::map<int, float> & posterior) const
  {
    std::vector<double> candidates;
    candidates.reserve(posterior.size());
    for (const auto & entry : posterior) {
      if (entry.first > 0 && entry.second > 0.0f) {
        candidates.push_back(entry.second);
      }
    }
    std::sort(candidates.begin(), candidates.end(), std::greater<double>());
    if (candidates.size() < 2 || candidates[1] <= std::numeric_limits<double>::epsilon()) {
      return std::numeric_limits<double>::infinity();
    }
    return candidates[0] / candidates[1];
  }

  bool localization_covariance_valid(const cv::Mat & covariance, std::string & reason) const
  {
    if (covariance.rows != 6 || covariance.cols != 6 || covariance.type() != CV_64FC1) {
      reason = "localization covariance is unavailable";
      return false;
    }
    const double x_variance = covariance.at<double>(0, 0);
    const double y_variance = covariance.at<double>(1, 1);
    const double yaw_variance = covariance.at<double>(5, 5);
    if (
      !std::isfinite(x_variance) || !std::isfinite(y_variance) ||
      !std::isfinite(yaw_variance) || x_variance < 0.0 || y_variance < 0.0 ||
      yaw_variance < 0.0)
    {
      reason = "localization covariance contains invalid values";
      return false;
    }
    if (std::max(x_variance, y_variance) > max_linear_variance_) {
      reason = "linear localization variance is too high";
      return false;
    }
    if (yaw_variance > max_yaw_variance_) {
      reason = "yaw localization variance is too high";
      return false;
    }
    return true;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped make_pose_message(
    const rtabmap::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    const cv::Mat & covariance,
    bool fallback) const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = stamp;
    rtabmap_conversions::transformToPoseMsg(pose, msg.pose.pose);
    msg.pose.covariance.fill(0.0);

    if (!fallback && covariance.rows == 6 && covariance.cols == 6 &&
      covariance.type() == CV_64FC1)
    {
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          msg.pose.covariance[static_cast<std::size_t>(row * 6 + col)] =
            covariance.at<double>(row, col);
        }
      }
    } else {
      msg.pose.covariance[0] = fallback_linear_variance_;
      msg.pose.covariance[7] = fallback_linear_variance_;
      msg.pose.covariance[14] = 1.0;
      msg.pose.covariance[21] = 1.0;
      msg.pose.covariance[28] = 1.0;
      msg.pose.covariance[35] = fallback_yaw_variance_;
    }
    return msg;
  }

  void publish_localization(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
    int matched_node_id,
    double score,
    bool fallback)
  {
    accepted_pose_msg_ = pose_msg;
    visual_pose_pub_->publish(accepted_pose_msg_);

    std_msgs::msg::Bool success_msg;
    success_msg.data = true;
    success_pub_->publish(success_msg);

    if (publish_initialpose_) {
      initialpose_remaining_ = initialpose_publish_repetitions_;
      publish_initialpose_once();
    }

    accepted_pose_ = rtabmap_conversions::transformFromPoseMsg(accepted_pose_msg_.pose.pose);
    pose_published_at_ = std::chrono::steady_clock::now();
    tf_confirmation_started_at_ = pose_published_at_;

    std::ostringstream detail;
    if (fallback) {
      detail << "published last database pose fallback";
    } else {
      detail << "matched database node " << matched_node_id << " score=" << score;
    }
    publish_status("localized", detail.str());
    RCLCPP_INFO(
      get_logger(), "Visual initial pose published: %s", accepted_pose_.prettyPrint().c_str());

    if (activate_nav2_on_success_) {
      state_ = State::WAITING_TF_CONFIRMATION;
    } else {
      state_ = State::COMPLETE;
    }
  }

  void publish_initialpose_once()
  {
    if (!publish_initialpose_ || initialpose_remaining_ <= 0) {
      return;
    }
    accepted_pose_msg_.header.stamp = now();
    last_internal_initialpose_stamp_ = accepted_pose_msg_.header.stamp;
    have_internal_initialpose_stamp_ = true;
    initialpose_pub_->publish(accepted_pose_msg_);
    --initialpose_remaining_;
    next_initialpose_publish_at_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(initialpose_publish_interval_sec_));
    RCLCPP_INFO(
      get_logger(), "Published %s (%d repetitions remaining).",
      initialpose_topic_.c_str(), initialpose_remaining_);
  }

  void timer_callback()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    if (initialpose_remaining_ > 0 && now_steady >= next_initialpose_publish_at_) {
      publish_initialpose_once();
    }

    if (state_ == State::SEARCHING) {
      handle_search_timeouts(now_steady);
      return;
    }
    if (state_ == State::WAITING_TF_CONFIRMATION) {
      handle_tf_confirmation(now_steady);
      return;
    }
    if (state_ == State::ACTIVATING_NAVIGATION) {
      drive_lifecycle_activation(
        navigation_client_, navigation_manager_service_, "Nav2 navigation", State::COMPLETE);
    }
  }

  void handle_search_timeouts(const std::chrono::steady_clock::time_point & now_steady)
  {
    if (!first_frame_received_) {
      if (!sensor_timeout_reported_ &&
        std::chrono::duration<double>(now_steady - started_at_).count() >=
        sensor_wait_timeout_sec_)
      {
        sensor_timeout_reported_ = true;
        publish_status("alarm", "no RGB-D data received before sensor timeout");
        RCLCPP_ERROR(
          get_logger(), "No RGB-D data received after %.1f seconds on %s.",
          sensor_wait_timeout_sec_, rgbd_topic_.c_str());
      }
      return;
    }

    if (!localization_timeout_reported_ &&
      std::chrono::duration<double>(now_steady - first_frame_at_).count() >=
      localization_timeout_sec_)
    {
      localization_timeout_reported_ = true;
      RCLCPP_WARN(
        get_logger(), "Visual localization timed out after %.1f seconds and %d attempts.",
        localization_timeout_sec_, attempt_count_);

      if (allow_last_pose_fallback_ && !database_last_pose_.isNull()) {
        rtabmap::Transform fallback_pose = database_last_pose_.clone();
        if (flatten_to_2d_) {
          float x, y, z, roll, pitch, yaw;
          fallback_pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
          fallback_pose = rtabmap::Transform(x, y, 0.0f, 0.0f, 0.0f, yaw);
        }
        publish_status("fallback", "using the last localization pose saved in the database");
        publish_localization(
          make_pose_message(fallback_pose, now(), cv::Mat(), true), 0, 0.0, true);
      } else if (continue_search_after_timeout_) {
        publish_status("alarm", "localization timed out; continuing visual search");
      } else {
        state_ = State::FAILED;
        publish_status("failed", "visual localization timed out");
      }
    }
  }

  void handle_tf_confirmation(const std::chrono::steady_clock::time_point & now_steady)
  {
    if (std::chrono::duration<double>(now_steady - pose_published_at_).count() <
      tf_settle_delay_sec_)
    {
      return;
    }

    try {
      const auto tf_msg = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const rtabmap::Transform current_pose =
        rtabmap_conversions::transformFromGeometryMsg(tf_msg.transform);
      float accepted_roll, accepted_pitch, accepted_yaw;
      float current_roll, current_pitch, current_yaw;
      accepted_pose_.getEulerAngles(accepted_roll, accepted_pitch, accepted_yaw);
      current_pose.getEulerAngles(current_roll, current_pitch, current_yaw);
      const double linear_error = std::hypot(
        static_cast<double>(current_pose.x() - accepted_pose_.x()),
        static_cast<double>(current_pose.y() - accepted_pose_.y()));
      const double yaw_error = std::abs(normalize_angle(current_yaw - accepted_yaw));

      if (linear_error <= tf_confirmation_linear_tolerance_ &&
        yaw_error <= tf_confirmation_yaw_tolerance_)
      {
        RCLCPP_INFO(
          get_logger(), "map->%s confirmed (linear error=%.3f m, yaw error=%.3f rad).",
          base_frame_.c_str(), linear_error, yaw_error);
        publish_status("tf_confirmed", "map to base transform agrees with visual initial pose");
        activation_started_at_ = now_steady;
        lifecycle_request_in_flight_ = false;
        state_ = State::ACTIVATING_NAVIGATION;
        return;
      }
    } catch (const tf2::TransformException &) {
      // The transform may not be available until RTAB-Map processes another frame.
    }

    if (std::chrono::duration<double>(now_steady - tf_confirmation_started_at_).count() >=
      tf_confirmation_timeout_sec_)
    {
      state_ = State::FAILED;
      publish_status("failed", "timed out waiting for map to base TF confirmation");
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for %s -> %s to reflect the visual initial pose.",
        map_frame_.c_str(), base_frame_.c_str());
    }
  }

  void drive_lifecycle_activation(
    const rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr & client,
    const std::string & service_name,
    const std::string & label,
    State success_state)
  {
    if (lifecycle_request_in_flight_) {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (!client->service_is_ready()) {
      if (std::chrono::duration<double>(now_steady - activation_started_at_).count() >=
        lifecycle_service_timeout_sec_)
      {
        state_ = State::FAILED;
        publish_status("failed", "lifecycle service unavailable: " + service_name);
        RCLCPP_ERROR(get_logger(), "Lifecycle service unavailable: %s", service_name.c_str());
      }
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    lifecycle_request_in_flight_ = true;
    RCLCPP_INFO(get_logger(), "Calling %s to activate %s.", service_name.c_str(), label.c_str());

    client->async_send_request(
      request,
      [this, label, success_state](
        rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
      {
        lifecycle_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (!response->success) {
            state_ = State::FAILED;
            publish_status("failed", "lifecycle manager failed to activate " + label);
            RCLCPP_ERROR(get_logger(), "Lifecycle manager failed to activate %s.", label.c_str());
            return;
          }
        } catch (const std::exception & error) {
          state_ = State::FAILED;
          publish_status("failed", "lifecycle call failed for " + label);
          RCLCPP_ERROR(
            get_logger(), "Lifecycle call failed for %s: %s", label.c_str(), error.what());
          return;
        }

        RCLCPP_INFO(get_logger(), "%s activated.", label.c_str());
        if (success_state == State::COMPLETE) {
          state_ = State::COMPLETE;
          publish_status("complete", "visual initial pose accepted and Nav2 activated");
        } else {
          state_ = success_state;
          activation_started_at_ = std::chrono::steady_clock::now();
        }
      });
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    const std::string text = "state=" + state + " detail=" + detail;
    if (text == last_status_) {
      return;
    }
    last_status_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  rtabmap::Rtabmap rtabmap_;
  bool database_initialized_{false};
  rtabmap::Transform database_last_pose_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    visual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr candidate_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr candidate_score_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;

  State state_{State::INITIALIZING};
  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point first_frame_at_;
  std::chrono::steady_clock::time_point last_process_at_;
  std::chrono::steady_clock::time_point pose_published_at_;
  std::chrono::steady_clock::time_point tf_confirmation_started_at_;
  std::chrono::steady_clock::time_point activation_started_at_;
  std::chrono::steady_clock::time_point next_initialpose_publish_at_;
  bool first_frame_received_{false};
  bool sensor_timeout_reported_{false};
  bool localization_timeout_reported_{false};
  bool lifecycle_request_in_flight_{false};
  bool have_internal_initialpose_stamp_{false};
  int attempt_count_{0};
  int confirmed_node_id_{0};
  int confirmation_count_{0};
  int initialpose_remaining_{0};
  std::string last_status_;
  geometry_msgs::msg::PoseWithCovarianceStamped accepted_pose_msg_;
  builtin_interfaces::msg::Time last_internal_initialpose_stamp_;
  rtabmap::Transform accepted_pose_;

  std::string database_path_;
  std::string rgbd_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string visual_pose_topic_;
  std::string initialpose_topic_;
  std::string status_topic_;
  std::string navigation_manager_service_;

  double process_interval_sec_{1.0};
  double sensor_wait_timeout_sec_{30.0};
  double localization_timeout_sec_{30.0};
  bool continue_search_after_timeout_{true};
  double tf_lookup_timeout_sec_{0.2};
  double min_hypothesis_{0.11};
  int min_visual_inliers_{15};
  double min_visual_inliers_ratio_{0.0};
  double min_best_second_ratio_{1.0};
  double max_optimization_error_ratio_{3.0};
  double max_linear_variance_{1.0};
  double max_yaw_variance_{1.0};
  int required_confirmations_{1};
  int detector_strategy_{-1};
  int feature_type_{-1};
  int max_features_{800};
  int estimation_type_{-1};
  bool flatten_to_2d_{true};
  bool publish_initialpose_{true};
  int initialpose_publish_repetitions_{3};
  double initialpose_publish_interval_sec_{0.2};
  double fallback_linear_variance_{0.25};
  double fallback_yaw_variance_{0.068};
  bool allow_last_pose_fallback_{false};
  bool activate_nav2_on_success_{false};
  double tf_confirmation_timeout_sec_{10.0};
  double tf_settle_delay_sec_{0.5};
  double tf_confirmation_linear_tolerance_{0.5};
  double tf_confirmation_yaw_tolerance_{0.5};
  double lifecycle_service_timeout_sec_{30.0};
};

}  // namespace visual_initial_pose

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visual_initial_pose::VisualInitialPoseNode>());
  rclcpp::shutdown();
  return 0;
}
