#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "vine_interfaces/msg/chassis_led.hpp"

class LedControlChassis : public rclcpp::Node {
public:
  using ChassisLed = vine_interfaces::msg::ChassisLed;

  LedControlChassis() : Node("led_control_chassis") {
    // Create a subscriber to the "chassis_led_control" topic
    subscription_ = this->create_subscription<ChassisLed>(
      "chassis_led_control", 10,
      std::bind(&LedControlChassis::led_control_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LedControlChassis node has been started.");
    RCLCPP_INFO(this->get_logger(), "Waiting for messages on 'chassis_led_control' topic...");
  }

private:
  void led_control_callback(const ChassisLed::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->brightness);
    call_script(std::to_string(msg->brightness));
  }

  void call_script(const std::string& args) {
    // Construct the command to call the script
    std::string command = "bash /home/workspace/demo.sh " + args;
    int result = system(command.c_str());

    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "Script executed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Script execution failed with code %d", result);
    }
  }

  rclcpp::Subscription<ChassisLed>::SharedPtr subscription_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedControlChassis>());
    rclcpp::shutdown();
    return 0;
}
