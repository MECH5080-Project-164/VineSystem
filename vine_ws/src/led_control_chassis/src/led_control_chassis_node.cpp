#include <string>
#include <vector>
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
    // Declare the node parameters
    auto pwm_pins_desc = rcl_interfaces::msg::ParameterDescriptor();
    pwm_pins_desc.description = "List of GPIO pins (BCM) for PWM output";
    this->declare_parameter<std::vector<int64_t>>("pwm_pins", {12}, pwm_pins_desc);
    this->declare_parameter<int64_t>("pwm_range", 1024);
    this->declare_parameter<int64_t>("pwm_clock", 1);

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
    std::vector<int64_t> pwm_pins_64 = this->get_parameter("pwm_pins").as_integer_array();
    // Convert int64_t to native int
    for (const auto& pin_64 : pwm_pins_64) {
      this->pwm_pins_.push_back(static_cast<int>(pin_64));
    }

    // Set up the GPIO pins for PWM output
    for (const auto& pin : this->pwm_pins_) {
      pinMode(pin, PWM_OUTPUT);
      RCLCPP_INFO(this->get_logger(), "Configured GPIO pin %d for PWM output", pin);
    }
    // Set the PWM mode, range, and clock
    this->pwm_range = static_cast<int>(this->get_parameter("pwm_range").as_int());
    this->pwm_clock = static_cast<int>(this->get_parameter("pwm_clock").as_int());

    if (pwm_range <= 0 || pwm_clock <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid PWM range or clock value");
      return;
    }

    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(this->pwm_range);
    pwmSetClock(this->pwm_clock);

    }

private:
  void led_control_callback(const ChassisLed::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->brightness);
    configure_pwm(msg->brightness);
  }

  void configure_pwm(const int brightness) {

    if (brightness < 0 || brightness > 100) {
      RCLCPP_ERROR(this->get_logger(), "Brightness value out of range: %d", brightness);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Setting brightness to %d", brightness);

    // Set the PWM value based on the brightness level
    int pwm_value = (brightness * this->pwm_range) / 100; // Scale to 0-1023
    for (const auto& pin : this->pwm_pins_) {
      pwmWrite(pin, pwm_value);
      RCLCPP_INFO(this->get_logger(), "Set GPIO pin %d to PWM value %d", pin, pwm_value);
    }
  }

#ifdef USE_SCRIPT
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
#endif  // USE_SCRIPT

  rclcpp::Subscription<ChassisLed>::SharedPtr subscription_;
  std::vector<int> pwm_pins_; // Store the GPIO pins for PWM output
  int pwm_range; // Store the PWM range
  int pwm_clock; // Store the PWM clock

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedControlChassis>());
    rclcpp::shutdown();
    return 0;
}
