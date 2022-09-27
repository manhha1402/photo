
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "photo_msgs/srv/capture.hpp"
#include "photo_msgs/srv/get_config.hpp"
#include "photo_msgs/srv/set_config.hpp"
#include <photo/photo.h>

using namespace std;

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_photo");
  rclcpp::Client<photo_msgs::srv::SetConfig>::SharedPtr client =
      node->create_client<photo_msgs::srv::SetConfig>("/photo/set_config");
  auto request = std::make_shared<photo_msgs::srv::SetConfig::Request>();
  request->param = "exptime";
  request->value = "20";
  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }
  auto result = client->async_send_request(request);
  //
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "valid: %ld", result.valid());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service set_config");
  }

  rclcpp::shutdown();
  return 0;
}
