// 独立调试工具节点：通过命令行一次性读取或设置禁行区
// 用法示例：
//   ros2 run forbidden_area_layer forbidden_area_server get
//   ros2 run forbidden_area_layer forbidden_area_server clear
//   ros2 run forbidden_area_layer forbidden_area_server set 0.0 0.0 2.0 2.0
//   ros2 run forbidden_area_layer forbidden_area_server polygon 0.0 0.0 2.0 0.0 1.5 1.5 0.0 2.0
//   ros2 run forbidden_area_layer forbidden_area_server polygons 0 0 2 0 1 1 -- 3 3 5 3 4 5
//
// "set" 参数格式：每 4 个值表示一个矩形 [x1 y1 x2 y2]，可重复追加多个矩形
// "polygon" 参数格式：每 2 个值表示一个顶点 [x y]，至少 3 个顶点
// "polygons" 参数格式：用 -- 分隔多个多边形，每个多边形至少 3 个顶点

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "forbidden_area_layer/msg/forbidden_area.hpp"
#include "forbidden_area_layer/srv/get_forbidden_areas.hpp"
#include "forbidden_area_layer/srv/set_forbidden_areas.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GetSrv = forbidden_area_layer::srv::GetForbiddenAreas;
using SetSrv = forbidden_area_layer::srv::SetForbiddenAreas;
using FaMsg = forbidden_area_layer::msg::ForbiddenArea;

static constexpr const char * kGetService =
  "/global_costmap/get_forbidden_areas";
static constexpr const char * kSetService =
  "/global_costmap/set_forbidden_areas";

static void print_usage()
{
  std::cerr <<
    "Usage:\n"
    "  forbidden_area_server get\n"
    "  forbidden_area_server clear\n"
    "  forbidden_area_server set x1 y1 x2 y2 [x1 y1 x2 y2 ...]\n"
    "  forbidden_area_server polygon x1 y1 x2 y2 x3 y3 [x4 y4 ...]\n"
    "  forbidden_area_server polygons x1 y1 x2 y2 x3 y3 [x4 y4 ...] -- x1 y1 x2 y2 x3 y3 ...\n";
}

static bool parse_double(const char * text, double & value)
{
  char * end = nullptr;
  value = std::strtod(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

static void append_point(FaMsg & area, double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  area.points.push_back(p);
}

static int do_get(rclcpp::Node::SharedPtr node)
{
  auto client = node->create_client<GetSrv>(kGetService);
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "%s service not available", kGetService);
    return 1;
  }
  auto future = client->async_send_request(std::make_shared<GetSrv::Request>());
  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    return 1;
  }
  auto response = future.get();
  auto & areas = response->areas;
  std::cout << "Forbidden areas (" << areas.size() << "):\n";
  for (size_t i = 0; i < areas.size(); ++i) {
    std::cout << "  [" << i << "]:";
    for (const auto & p : areas[i].points) {
      std::cout << " (" << p.x << "," << p.y << ")";
    }
    std::cout << "\n";
  }
  return 0;
}

static int send_areas(rclcpp::Node::SharedPtr node, const std::vector<FaMsg> & areas)
{
  auto client = node->create_client<SetSrv>(kSetService);
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "%s service not available", kSetService);
    return 1;
  }

  auto req = std::make_shared<SetSrv::Request>();
  req->areas = areas;

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    return 1;
  }
  auto response = future.get();
  std::cout << (response->success ? "OK" : "FAIL") << ": " << response->message << "\n";
  return response->success ? 0 : 1;
}

static int do_set(rclcpp::Node::SharedPtr node, const std::vector<double> & coords)
{
  std::vector<FaMsg> areas;
  for (size_t i = 0; i + 3 < coords.size(); i += 4) {
    double x1 = coords[i], y1 = coords[i + 1], x2 = coords[i + 2], y2 = coords[i + 3];
    FaMsg fa;
    append_point(fa, x1, y1);
    append_point(fa, x2, y1);
    append_point(fa, x2, y2);
    append_point(fa, x1, y2);
    areas.push_back(fa);
  }

  return send_areas(node, areas);
}

static int do_polygon(rclcpp::Node::SharedPtr node, const std::vector<double> & coords)
{
  FaMsg area;
  for (size_t i = 0; i + 1 < coords.size(); i += 2) {
    append_point(area, coords[i], coords[i + 1]);
  }
  return send_areas(node, {area});
}

static int do_polygons(rclcpp::Node::SharedPtr node, const std::vector<std::vector<double>> & polygons)
{
  std::vector<FaMsg> areas;
  for (const auto & coords : polygons) {
    FaMsg area;
    for (size_t i = 0; i + 1 < coords.size(); i += 2) {
      append_point(area, coords[i], coords[i + 1]);
    }
    areas.push_back(area);
  }
  return send_areas(node, areas);
}

static bool parse_coords(int argc, char ** argv, int start, std::vector<double> & coords)
{
  for (int i = start; i < argc; ++i) {
    double value = 0.0;
    if (!parse_double(argv[i], value)) {
      std::cerr << "Invalid number: " << argv[i] << "\n";
      return false;
    }
    coords.push_back(value);
  }
  return true;
}

static bool parse_polygon_groups(
  int argc, char ** argv, int start, std::vector<std::vector<double>> & polygons)
{
  std::vector<double> current;
  for (int i = start; i < argc; ++i) {
    if (std::strcmp(argv[i], "--") == 0) {
      if (current.size() < 6 || current.size() % 2 != 0) {
        std::cerr << "Each polygon requires at least 3 x/y point pairs\n";
        return false;
      }
      polygons.push_back(current);
      current.clear();
      continue;
    }

    double value = 0.0;
    if (!parse_double(argv[i], value)) {
      std::cerr << "Invalid number: " << argv[i] << "\n";
      return false;
    }
    current.push_back(value);
  }

  if (current.size() < 6 || current.size() % 2 != 0) {
    std::cerr << "Each polygon requires at least 3 x/y point pairs\n";
    return false;
  }
  polygons.push_back(current);

  return true;
}

static int shutdown_with(rclcpp::Node::SharedPtr node, int ret)
{
  node.reset();
  rclcpp::shutdown();
  return ret;
}

static bool valid_rectangle_coords(const std::vector<double> & coords)
{
  if (coords.size() < 4 || coords.size() % 4 != 0) {
    std::cerr << "set requires multiples of 4 values: x1 y1 x2 y2 ...\n";
    return false;
  }
  return true;
}

static bool valid_polygon_coords(const std::vector<double> & coords)
{
  if (coords.size() < 6 || coords.size() % 2 != 0) {
    std::cerr << "polygon requires at least 3 x/y point pairs\n";
    return false;
  }
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("forbidden_area_tool");

  if (argc < 2) {
    print_usage();
    return shutdown_with(node, 1);
  }

  std::string cmd(argv[1]);
  int ret = 0;

  if (cmd == "get") {
    ret = do_get(node);
  } else if (cmd == "clear") {
    ret = send_areas(node, {});
  } else if (cmd == "set") {
    std::vector<double> coords;
    if (!parse_coords(argc, argv, 2, coords) || !valid_rectangle_coords(coords)) {
      return shutdown_with(node, 1);
    }
    ret = do_set(node, coords);
  } else if (cmd == "polygon") {
    std::vector<double> coords;
    if (!parse_coords(argc, argv, 2, coords) || !valid_polygon_coords(coords)) {
      return shutdown_with(node, 1);
    }
    ret = do_polygon(node, coords);
  } else if (cmd == "polygons") {
    std::vector<std::vector<double>> polygons;
    if (!parse_polygon_groups(argc, argv, 2, polygons)) {
      return shutdown_with(node, 1);
    }
    ret = do_polygons(node, polygons);
  } else {
    std::cerr << "Unknown command: " << cmd << "\n";
    print_usage();
    ret = 1;
  }

  return shutdown_with(node, ret);
}
