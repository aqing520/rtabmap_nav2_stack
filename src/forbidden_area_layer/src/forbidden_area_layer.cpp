#include "forbidden_area_layer/forbidden_area_layer.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "forbidden_area_layer/msg/forbidden_area.hpp"
#include "forbidden_area_layer/srv/get_forbidden_areas.hpp"
#include "forbidden_area_layer/srv/set_forbidden_areas.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace forbidden_area_layer
{

void ForbiddenAreaLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // 平坦 double 数组，每 4 个一组 [x1, y1, x2, y2] 表示一个矩形禁行区
  declareParameter("forbidden_areas", rclcpp::ParameterValue(std::vector<double>{}));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + ".enabled", enabled_);

  std::vector<double> areas_flat;
  node->get_parameter(name_ + ".forbidden_areas", areas_flat);
  {
    std::lock_guard<std::mutex> lock(areas_mutex_);
    parseRectangles(areas_flat);
  }

  // Service：获取当前禁行区列表
  get_areas_service_ = node->create_service<srv::GetForbiddenAreas>(
    "get_forbidden_areas",
    [this](
      const std::shared_ptr<srv::GetForbiddenAreas::Request> /*req*/,
      std::shared_ptr<srv::GetForbiddenAreas::Response> resp)
    {
      std::lock_guard<std::mutex> lock(areas_mutex_);
      for (const auto & polygon : forbidden_areas_) {
        msg::ForbiddenArea area_msg;
        for (const auto & pt : polygon) {
          geometry_msgs::msg::Point p;
          p.x = pt.x;
          p.y = pt.y;
          p.z = 0.0;
          area_msg.points.push_back(p);
        }
        resp->areas.push_back(area_msg);
      }
    });

  // Service：设置新的禁行区列表（支持任意多边形，至少 3 个顶点）
  set_areas_service_ = node->create_service<srv::SetForbiddenAreas>(
    "set_forbidden_areas",
    [this](
      const std::shared_ptr<srv::SetForbiddenAreas::Request> req,
      std::shared_ptr<srv::SetForbiddenAreas::Response> resp)
    {
      std::vector<Polygon> new_areas;
      for (const auto & area_msg : req->areas) {
        if (area_msg.points.size() < 3) {
          continue;
        }
        Polygon polygon;
        for (const auto & p : area_msg.points) {
          polygon.push_back({p.x, p.y});
        }
        new_areas.push_back(polygon);
      }
      {
        std::lock_guard<std::mutex> lock(areas_mutex_);
        forbidden_areas_ = std::move(new_areas);
        current_ = false;
      }
      resp->success = true;
      resp->message =
        "Updated " + std::to_string(forbidden_areas_.size()) + " forbidden areas";
    });

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ForbiddenAreaLayer::dynamicParametersCallback,
      this, std::placeholders::_1));

  current_ = true;
}

void ForbiddenAreaLayer::parseRectangles(const std::vector<double> & flat)
{
  forbidden_areas_.clear();
  // 每 4 个值 [x1, y1, x2, y2] 生成一个矩形（4 顶点，逆时针）
  for (size_t i = 0; i + 3 < flat.size(); i += 4) {
    double x1 = flat[i], y1 = flat[i + 1], x2 = flat[i + 2], y2 = flat[i + 3];
    forbidden_areas_.push_back({{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}});
  }
}

void ForbiddenAreaLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    current_ = true;
    return;
  }

  std::lock_guard<std::mutex> lock(areas_mutex_);
  if (forbidden_areas_.empty()) {
    current_ = true;
    return;
  }

  for (const auto & area : forbidden_areas_) {
    for (const auto & pt : area) {
      // 等价于 CostmapLayer::touch()，手动扩展 bounds
      *min_x = std::min(*min_x, pt.x);
      *min_y = std::min(*min_y, pt.y);
      *max_x = std::max(*max_x, pt.x);
      *max_y = std::max(*max_y, pt.y);
    }
  }
}

void ForbiddenAreaLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    current_ = true;
    return;
  }

  std::lock_guard<std::mutex> lock(areas_mutex_);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx, wy;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i),
        static_cast<unsigned int>(j),
        wx, wy);
      if (isPointInForbiddenArea(wx, wy)) {
        master_grid.setCost(i, j, LETHAL_OBSTACLE);
      }
    }
  }

  current_ = true;
}

bool ForbiddenAreaLayer::isPointInForbiddenArea(double x, double y)
{
  for (const auto & area : forbidden_areas_) {
    if (isPointInPolygon({x, y}, area)) {
      return true;
    }
  }
  return false;
}

// 射线法判断点是否在多边形内
bool ForbiddenAreaLayer::isPointInPolygon(const Point & point, const Polygon & polygon)
{
  bool inside = false;
  size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
      (point.x <
      (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
      (polygon[j].y - polygon[i].y) + polygon[i].x))
    {
      inside = !inside;
    }
  }
  return inside;
}

rcl_interfaces::msg::SetParametersResult
ForbiddenAreaLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto & parameter : parameters) {
    const auto & param_name = parameter.get_name();
    if (param_name == name_ + ".enabled") {
      enabled_ = parameter.as_bool();
      current_ = false;
    } else if (param_name == name_ + ".forbidden_areas") {
      std::lock_guard<std::mutex> lock(areas_mutex_);
      parseRectangles(parameter.as_double_array());
      current_ = false;
    }
  }

  result.successful = true;
  return result;
}

}  // namespace forbidden_area_layer

PLUGINLIB_EXPORT_CLASS(forbidden_area_layer::ForbiddenAreaLayer, nav2_costmap_2d::Layer)
