#ifndef NAV2_CONTROLLER_BENCHMARK__TIMING_CONTROLLER_HPP_
#define NAV2_CONTROLLER_BENCHMARK__TIMING_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_controller_benchmark
{

class TimingController : public nav2_core::Controller
{
public:
  TimingController();
  ~TimingController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void publishDuration(double duration_ms);

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr wrapped_controller_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("TimingController")};
  std::string plugin_name_;
  std::string wrapped_plugin_;
  std::string timing_topic_;
};

}  // namespace nav2_controller_benchmark

#endif  // NAV2_CONTROLLER_BENCHMARK__TIMING_CONTROLLER_HPP_
