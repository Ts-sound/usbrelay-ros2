#include <cstdio>
#include <memory>
#include <string>

#include "libusbrelay.h"
#include "rclcpp/rclcpp.hpp"
#include "usbrelay_interfaces/srv/get_relay.hpp"
#include "usbrelay_interfaces/srv/set_relay.hpp"
#include "utils.h"

std::string board_path;

void InitRelay() {
  //
  relay_board *board = nullptr;

  {
    int ret = 0;

    ret = enumerate_relay_boards("", 1, 1);
    std::cout << "---------------------------------" << ret << std::endl;
  }

  {
    int cnt = get_relay_board_count();
    std::cout << "---------------------------------" << std::endl;

    auto boards = get_relay_boards();
    for (int i = 0; i < cnt; i++) {
      std::cout << "\t path : " << boards[i].path
                << "\t type : " << (int)(boards[i].module_type)
                << "\t relay_cnt : " << (int)boards[i].relay_count
                << "\t serial : " << (char *)&(boards[i].serial)
                << "\t state : " << (int)boards[i].state << std::endl;
      if (board_path == boards[i].path) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "board %s found ",
                    board_path.c_str());
        board = &(boards[i]);
      }
    }
    std::cout << "---------------------------------" << std::endl;

    if (board == nullptr) {
      shutdown();
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "relay board %s not found ",
                   board_path.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "exit ");
      rclcpp::shutdown();
      raise(SIGINT);
    }
  }
}

void HandleGetRelay(
    const std::shared_ptr<usbrelay_interfaces::srv::GetRelay::Request>
        request,  // CHANGE
    std::shared_ptr<usbrelay_interfaces::srv::GetRelay::Response>
        response)  // CHANGE
{
  request->req;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "req: %d", request->req);

  relay_board *board = nullptr;
  enumerate_relay_boards("", 1, 0);
  board = find_board(board_path.c_str(), 0);
  uint8_t state = 0;
  if (board) {
    state = board->state;
    for (size_t i = 0; i < board->relay_count; i++) {
      response->input.push_back(GetBit(state, i));
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "board  %s not found",
                 board_path.c_str());
  }

  // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s",
              ToString(response->input).c_str());
}

void HandleSetRelay(
    const std::shared_ptr<usbrelay_interfaces::srv::SetRelay::Request>
        request,  // CHANGE
    std::shared_ptr<usbrelay_interfaces::srv::SetRelay::Response>
        response)  // CHANGE
{
  response->result = false;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set relay: %s",
              ToString(request->output).c_str());

  auto board = find_board(board_path.c_str(), 0);

  if (!board) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), " %s not found",
                 board_path.c_str());
    return;
  }

  //
  auto size = std::min(request->output.size(), (size_t)board->relay_count);
  if (size < request->output.size()) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                " relay count is %u , less than output size %lu",
                board->relay_count, request->output.size());
  }

  for (size_t i = 0; i < size; i++) {
    int ret = -1;
    if (request->output[i]) {
      ret = operate_relay(board_path.c_str(), i + 1, CMD_ON, 0);
    } else {
      ret = operate_relay(board_path.c_str(), i + 1, CMD_OFF, 0);
    }
    if (ret == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), " operate relay %lu faild",
                   i + 1);
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("usbrelay");  // CHANGE

  node->declare_parameter<std::string>("board_path", "/dev/hidraw0");
  board_path = node->get_parameter("board_path").as_string();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "load params board_path : %s ",
              board_path.c_str());

  auto service =  // CHANGE
      node->create_service<usbrelay_interfaces::srv::GetRelay>(
          "get_relay", &HandleGetRelay);  // CHANGE
  auto service2 =                         // CHANGE
      node->create_service<usbrelay_interfaces::srv::SetRelay>(
          "set_relay", &HandleSetRelay);  // CHANGE

  InitRelay();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "usbrelay service start !!!");  // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
