#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "wiringPi.h"
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

    // Initialize wiringPi
    if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize wiringPi");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "wiringPi initialized successfully");

    // Configure GPIO pins
    pinMode(18, PWM_OUTPUT); // Set GPIO BMC pin 18 as PWM output
    pwmSetMode(PWM_MODE_MS); // Set PWM mode to mark-space
    pwmSetRange(1024); // Set PWM range to 1024
    pwmSetClock(1); // Set the PWM clock divisor to 1 - 19.2kHz
    pwmWrite(18, 0); // Initialize PWM value to 0
    RCLCPP_INFO(this->get_logger(), "GPIO pins configured successfully");
  }

private:
  void led_control_callback(const ChassisLed::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->brightness);
    configure_pwm(msg->brightness);
  }

  void configure_pwm(const int brightness) {

   int pwm_range = 1024; // Set the PWM range
    if (brightness < 0 || brightness > 100) {
      RCLCPP_ERROR(this->get_logger(), "Brightness value out of range: %d", brightness);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Setting brightness to %d", brightness);

    // Set the PWM value based on the brightness level
    int pwm_value = (brightness * pwm_range) / 100; // Scale to 0-1023
    pwmWrite(18, pwm_value); // Write to GPIO BCM pin 18
  }

  void call_script(const int brightness) {
    // Construct the command to call the script
    std::string command = "bash /home/workspace/demo.sh " + std::to_string(brightness);
    int result = system(command.c_str());

    RCLCPP_INFO(this->get_logger(), "Brightness value sent to script: %d", brightness);

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
