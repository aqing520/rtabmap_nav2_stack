#ifndef FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_TOOL_HPP_
#define FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_TOOL_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OgreVector3.h>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "forbidden_area_layer/srv/get_forbidden_areas.hpp"
#include "forbidden_area_layer/srv/set_forbidden_areas.hpp"

namespace rviz_rendering
{
class ViewportProjectionFinder;
}  // namespace rviz_rendering

namespace forbidden_area_layer
{

class ForbiddenAreaTool : public rviz_common::Tool
{
public:
  ForbiddenAreaTool();
  ~ForbiddenAreaTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void update(float wall_dt, float ros_dt) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  using GetForbiddenAreas = forbidden_area_layer::srv::GetForbiddenAreas;
  using SetForbiddenAreas = forbidden_area_layer::srv::SetForbiddenAreas;

  std::pair<bool, Ogre::Vector3> getMapPoint(rviz_common::ViewportMouseEvent & event) const;
  void publishPreview(const Ogre::Vector3 & first, const Ogre::Vector3 & second);
  void clearPreview();
  void clearAreasOnRvizStart();
  void appendRectangle(double x1, double y1, double x2, double y2);
  static std::vector<geometry_msgs::msg::Point> makeRectangle(
    double x1, double y1, double x2, double y2);

  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<GetForbiddenAreas>::SharedPtr get_areas_client_;
  rclcpp::Client<SetForbiddenAreas>::SharedPtr set_areas_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preview_publisher_;

  Ogre::Vector3 drag_start_;
  bool dragging_{false};
  bool request_in_flight_{false};
  bool clear_pending_{true};
  std::string frame_id_{"map"};
};

}  // namespace forbidden_area_layer

#endif  // FORBIDDEN_AREA_LAYER__FORBIDDEN_AREA_TOOL_HPP_
