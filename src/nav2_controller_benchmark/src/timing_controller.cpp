#include "nav2_controller_benchmark/timing_controller.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_benchmark
{

TimingController::TimingController()
: loader_("nav2_core", "nav2_core::Controller")
{
}

void TimingController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("TimingController parent node expired during configure");
  }

  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wrapped_plugin", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".timing_topic",
    rclcpp::ParameterValue(std::string("controller_benchmark/compute_time_ms")));

  wrapped_plugin_ = node->get_parameter(plugin_name_ + ".wrapped_plugin").as_string();
  timing_topic_ = node->get_parameter(plugin_name_ + ".timing_topic").as_string();

  if (wrapped_plugin_ == "nav2_controller_benchmark::TimingController") {
    throw std::runtime_error("TimingController cannot wrap itself");
  }

  wrapped_controller_ = loader_.createUniqueInstance(wrapped_plugin_);
  wrapped_controller_->configure(parent, plugin_name_, std::move(tf), std::move(costmap_ros));
  duration_pub_ = node->create_publisher<std_msgs::msg::Float64>(timing_topic_, 100);

  RCLCPP_INFO(
    logger_, "Timing controller '%s' wraps '%s'; publishing milliseconds on '%s'",
    plugin_name_.c_str(), wrapped_plugin_.c_str(), timing_topic_.c_str());
}

void TimingController::cleanup()
{
  if (wrapped_controller_) {
    wrapped_controller_->cleanup();
    wrapped_controller_.reset();
  }
  duration_pub_.reset();
}

void TimingController::activate()
{
  wrapped_controller_->activate();
}

void TimingController::deactivate()
{
  wrapped_controller_->deactivate();
}

void TimingController::setPlan(const nav_msgs::msg::Path & path)
{
  wrapped_controller_->setPlan(path);
}

geometry_msgs::msg::TwistStamped TimingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto start = std::chrono::steady_clock::now();
  try {
    auto command = wrapped_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    const auto stop = std::chrono::steady_clock::now();
    publishDuration(std::chrono::duration<double, std::milli>(stop - start).count());
    return command;
  } catch (...) {
    const auto stop = std::chrono::steady_clock::now();
    publishDuration(std::chrono::duration<double, std::milli>(stop - start).count());
    throw;
  }
}

void TimingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  wrapped_controller_->setSpeedLimit(speed_limit, percentage);
}

void TimingController::publishDuration(double duration_ms)
{
  if (!duration_pub_) {
    return;
  }
  std_msgs::msg::Float64 msg;
  msg.data = duration_ms;
  duration_pub_->publish(msg);
}

}  // namespace nav2_controller_benchmark

PLUGINLIB_EXPORT_CLASS(
  nav2_controller_benchmark::TimingController,
  nav2_core::Controller)
