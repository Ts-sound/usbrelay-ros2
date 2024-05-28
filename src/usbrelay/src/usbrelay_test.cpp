#include "rclcpp/rclcpp.hpp"
#include "usbrelay_interfaces/srv/get_relay.hpp" // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

#include "utils.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("usbrelay_test");       // CHANGE
    rclcpp::Client<usbrelay_interfaces::srv::GetRelay>::SharedPtr client =                 // CHANGE
        node->create_client<usbrelay_interfaces::srv::GetRelay>("get_relay"); // CHANGE

    auto request = std::make_shared<usbrelay_interfaces::srv::GetRelay::Request>(); // CHANGE
    request->req = true;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ret : %s ", ToString(result.get()->input).c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed "); // CHANGE
    }

    rclcpp::shutdown();
    return 0;
}