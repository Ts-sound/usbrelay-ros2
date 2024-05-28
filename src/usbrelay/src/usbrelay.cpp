#include <cstdio>
#include "usbrelay_interfaces/srv/get_relay.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include "utils.h"


void HandleGetRelay(const std::shared_ptr<usbrelay_interfaces::srv::GetRelay::Request> request, // CHANGE
                    std::shared_ptr<usbrelay_interfaces::srv::GetRelay::Response> response)     // CHANGE
{
  request->req;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "req: %d", request->req);
  // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", ToString(response->input).c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("usbrelay"); // CHANGE

  auto service =                                                                              // CHANGE
      node->create_service<usbrelay_interfaces::srv::GetRelay>("get_relay", &HandleGetRelay); // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usbrelay service start !!!"); // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
