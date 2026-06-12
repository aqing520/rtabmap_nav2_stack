#ifndef FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_LAYER_HPP_
#define FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_LAYER_HPP_

#include <algorithm>
#include <mutex>
#include <vector>

#include <nav2_costmap_2d/layer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace forbidden_area_layer
{

struct Point { double x, y; };
using Polygon = std::vector<Point>;

class ForbiddenAreaLayer : public nav2_costmap_2d::Layer
{
public:
  ForbiddenAreaLayer() = default;
  ~ForbiddenAreaLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override { current_ = false; }
  bool isClearable() override { return false; }

private:
  void parseRectangles(const std::vector<double> & flat);
  bool isPointInForbiddenArea(double x, double y);
  bool isPointInPolygon(const Point & point, const Polygon & polygon);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  std::vector<Polygon> forbidden_areas_;
  std::mutex areas_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // 用基类类型持有 service，避免头文件依赖生成的 srv 类型
  rclcpp::ServiceBase::SharedPtr get_areas_service_;
  rclcpp::ServiceBase::SharedPtr set_areas_service_;
};

}  // namespace forbidden_area_layer

#endif  // FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_LAYER_HPP_
