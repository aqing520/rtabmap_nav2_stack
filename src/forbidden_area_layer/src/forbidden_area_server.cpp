// 独立调试工具节点：通过命令行一次性读取或设置禁行区
// 用法示例：
//   ros2 run forbidden_area_layer forbidden_area_server get
//   ros2 run forbidden_area_layer forbidden_area_server set 0.0 0.0 2.0 2.0
//
// "set" 参数格式：每 4 个值表示一个矩形 [x1 y1 x2 y2]，可重复追加多个矩形

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
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
  auto & areas = future.get()->areas;
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

static int do_set(rclcpp::Node::SharedPtr node, const std::vector<double> & coords)
{
  auto client = node->create_client<SetSrv>(kSetService);
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "%s service not available", kSetService);
    return 1;
  }

  auto req = std::make_shared<SetSrv::Request>();
  for (size_t i = 0; i + 3 < coords.size(); i += 4) {
    double x1 = coords[i], y1 = coords[i + 1], x2 = coords[i + 2], y2 = coords[i + 3];
    FaMsg fa;
    geometry_msgs::msg::Point p;
    auto add = [&](double x, double y) {
      p.x = x; p.y = y; p.z = 0.0;
      fa.points.push_back(p);
    };
    add(x1, y1); add(x2, y1); add(x2, y2); add(x1, y2);
    req->areas.push_back(fa);
  }

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    return 1;
  }
  auto & res = *future.get();
  std::cout << (res.success ? "OK" : "FAIL") << ": " << res.message << "\n";
  return res.success ? 0 : 1;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("forbidden_area_tool");

  if (argc < 2) {
    std::cerr << "Usage: forbidden_area_server <get|set> [x1 y1 x2 y2 ...]\n";
    return 1;
  }

  std::string cmd(argv[1]);
  int ret = 0;

  if (cmd == "get") {
    ret = do_get(node);
  } else if (cmd == "set") {
    std::vector<double> coords;
    for (int i = 2; i < argc; ++i) {
      coords.push_back(std::atof(argv[i]));
    }
    if (coords.size() < 4 || coords.size() % 4 != 0) {
      std::cerr << "set requires multiples of 4 values: x1 y1 x2 y2 ...\n";
      return 1;
    }
    ret = do_set(node, coords);
  } else {
    std::cerr << "Unknown command: " << cmd << "\n";
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
