#include "forbidden_area_layer/forbidden_area_tool.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include <QCursor>
#include <QIcon>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include "forbidden_area_layer/msg/forbidden_area.hpp"

namespace forbidden_area_layer
{

namespace
{
constexpr double kMinimumRectangleSize = 0.02;
constexpr char kGetService[] = "/global_costmap/get_forbidden_areas";
constexpr char kSetService[] = "/global_costmap/set_forbidden_areas";
constexpr char kPreviewTopic[] = "/forbidden_area_preview";
}  // namespace

ForbiddenAreaTool::ForbiddenAreaTool()
{
  shortcut_key_ = 'f';
  setName("Add Forbidden Area");
  setDescription("Drag on the map to add a rectangular forbidden area.");
  setCursor(QCursor(Qt::CrossCursor));
  setIcon(QIcon::fromTheme("draw-rectangle"));
}

void ForbiddenAreaTool::onInitialize()
{
  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setStatus("RViz ROS node is unavailable.");
    return;
  }

  node_ = node_abstraction->get_raw_node();
  get_areas_client_ = node_->create_client<GetForbiddenAreas>(kGetService);
  set_areas_client_ = node_->create_client<SetForbiddenAreas>(kSetService);
  preview_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    kPreviewTopic, rclcpp::QoS(1).transient_local());
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

void ForbiddenAreaTool::activate()
{
  if (clear_pending_ || request_in_flight_) {
    setStatus("Waiting for the temporary forbidden areas to be cleared.");
  } else {
    setStatus("Drag with the left mouse button to create a rectangular forbidden area.");
  }
}

void ForbiddenAreaTool::deactivate()
{
  dragging_ = false;
  clearPreview();
}

void ForbiddenAreaTool::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (clear_pending_ && !request_in_flight_ && set_areas_client_ &&
    set_areas_client_->service_is_ready())
  {
    clearAreasOnRvizStart();
  }
}

int ForbiddenAreaTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!projection_finder_) {
    return Render;
  }

  const auto map_point = getMapPoint(event);

  if (event.leftDown()) {
    if (!map_point.first) {
      setStatus("Unable to project the cursor onto the map plane.");
      return Render;
    }
    drag_start_ = map_point.second;
    dragging_ = true;
    publishPreview(drag_start_, drag_start_);
    setStatus("Release the left mouse button to add the forbidden area.");
    return Render;
  }

  if (dragging_ && event.left() && map_point.first) {
    publishPreview(drag_start_, map_point.second);
    return Render;
  }

  if (dragging_ && event.leftUp()) {
    dragging_ = false;
    clearPreview();
    if (!map_point.first) {
      setStatus("Unable to project the cursor onto the map plane.");
      return Render | Finished;
    }

    const double width = std::abs(map_point.second.x - drag_start_.x);
    const double height = std::abs(map_point.second.y - drag_start_.y);
    if (width < kMinimumRectangleSize || height < kMinimumRectangleSize) {
      setStatus("The forbidden area is too small. Drag at least 2 cm in both directions.");
      return Render | Finished;
    }

    appendRectangle(drag_start_.x, drag_start_.y, map_point.second.x, map_point.second.y);
    return Render | Finished;
  }

  return Render;
}

std::pair<bool, Ogre::Vector3> ForbiddenAreaTool::getMapPoint(
  rviz_common::ViewportMouseEvent & event) const
{
  return projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
}

std::vector<geometry_msgs::msg::Point> ForbiddenAreaTool::makeRectangle(
  double x1, double y1, double x2, double y2)
{
  const double min_x = std::min(x1, x2);
  const double max_x = std::max(x1, x2);
  const double min_y = std::min(y1, y2);
  const double max_y = std::max(y1, y2);

  std::vector<geometry_msgs::msg::Point> points(4);
  points[0].x = min_x;
  points[0].y = min_y;
  points[1].x = max_x;
  points[1].y = min_y;
  points[2].x = max_x;
  points[2].y = max_y;
  points[3].x = min_x;
  points[3].y = max_y;
  return points;
}

void ForbiddenAreaTool::publishPreview(const Ogre::Vector3 & first, const Ogre::Vector3 & second)
{
  if (!preview_publisher_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "forbidden_area_preview";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.04;
  marker.color.r = 1.0F;
  marker.color.g = 0.75F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;
  marker.pose.orientation.w = 1.0;

  const auto points = makeRectangle(first.x, first.y, second.x, second.y);
  marker.points = points;
  marker.points.push_back(points.front());
  preview_publisher_->publish(marker);
}

void ForbiddenAreaTool::clearPreview()
{
  if (!preview_publisher_ || !node_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "forbidden_area_preview";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  preview_publisher_->publish(marker);
}

void ForbiddenAreaTool::clearAreasOnRvizStart()
{
  request_in_flight_ = true;
  auto request = std::make_shared<SetForbiddenAreas::Request>();
  set_areas_client_->async_send_request(
    request,
    [this](rclcpp::Client<SetForbiddenAreas>::SharedFuture future) {
      request_in_flight_ = false;
      try {
        const auto response = future.get();
        clear_pending_ = !response->success;
        setStatus(
          response->success ?
          "Temporary forbidden areas cleared. Drag to create a new area." :
          QString::fromStdString(response->message));
      } catch (const std::exception & error) {
        setStatus(QString("Failed to clear temporary forbidden areas: %1").arg(error.what()));
      }
    });
}

void ForbiddenAreaTool::appendRectangle(double x1, double y1, double x2, double y2)
{
  if (clear_pending_ || request_in_flight_) {
    setStatus("Waiting for the temporary forbidden areas to be cleared.");
    return;
  }
  if (!get_areas_client_->service_is_ready() || !set_areas_client_->service_is_ready()) {
    setStatus("Forbidden-area service is unavailable. Start Nav2 with nav2_forbidden_area.yaml.");
    return;
  }

  request_in_flight_ = true;
  const auto rectangle = makeRectangle(x1, y1, x2, y2);
  auto get_request = std::make_shared<GetForbiddenAreas::Request>();
  get_areas_client_->async_send_request(
    get_request,
    [this, rectangle](rclcpp::Client<GetForbiddenAreas>::SharedFuture future) {
      request_in_flight_ = false;

      GetForbiddenAreas::Response::SharedPtr get_response;
      try {
        get_response = future.get();
      } catch (const std::exception & error) {
        setStatus(QString("Failed to read existing forbidden areas: %1").arg(error.what()));
        return;
      }

      auto set_request = std::make_shared<SetForbiddenAreas::Request>();
      set_request->areas = get_response->areas;
      forbidden_area_layer::msg::ForbiddenArea new_area;
      new_area.points = rectangle;
      set_request->areas.push_back(new_area);

      request_in_flight_ = true;
      set_areas_client_->async_send_request(
        set_request,
        [this](rclcpp::Client<SetForbiddenAreas>::SharedFuture set_future) {
          request_in_flight_ = false;
          try {
            const auto response = set_future.get();
            setStatus(QString::fromStdString(response->message));
          } catch (const std::exception & error) {
            setStatus(QString("Failed to add forbidden area: %1").arg(error.what()));
          }
        });
    });
}

}  // namespace forbidden_area_layer

PLUGINLIB_EXPORT_CLASS(forbidden_area_layer::ForbiddenAreaTool, rviz_common::Tool)
