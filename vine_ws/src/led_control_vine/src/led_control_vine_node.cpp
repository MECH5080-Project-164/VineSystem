#include <chrono>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "wiringPi.h"
#include "vine_interfaces/msg/vine_led.hpp"

std::string to_string(const vine_interfaces::msg::VineLed::SharedPtr msg) {
  return "brightness: " + std::to_string(msg->brightness) +
         ", duration: " + std::to_string(msg->duration) +
         ", force: "    + std::to_string(msg->force);
}

class LedControlVine : public rclcpp::Node {
public:
  using VineLed = vine_interfaces::msg::VineLed;

  LedControlVine() : Node("led_control_vine") {
    // Declare the node parameters
    auto pwm_pins_desc = rcl_interfaces::msg::ParameterDescriptor();
    pwm_pins_desc.description = "List of GPIO pins (BCM) for PWM output";
    this->declare_parameter<std::vector<int64_t>>("pwm_pins", {13}, pwm_pins_desc);
    this->declare_parameter<int64_t>("pwm_range", 499); // For 50kHz PWM frequency
    this->declare_parameter<int64_t>("pwm_clock", 1); // For 50kHz PWM frequency

    // Create a subscriber to the "led_control/vine" topic
    subscription_ = this->create_subscription<VineLed>(
      "led_control/vine", 10,
      std::bind(&LedControlVine::led_control_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LedControlVine node has been started.");
    RCLCPP_INFO(this->get_logger(), "Waiting for messages on 'led_control/vine' topic...");

    // Initialise wiringPi
    if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize wiringPi");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "wiringPi initialized successfully");

    // Configure GPIO pins
    std::vector<int64_t> pwm_pins_64 = this->get_parameter("pwm_pins").as_integer_array();
    // Convert int64_t to native int
    for (const auto& pin_64 : pwm_pins_64) {
      this->pwm_pins_.push_back(static_cast<int>(pin_64));
    }

    // Set up the GPIO pins for PWM output
    for (const auto& pin : this->pwm_pins_) {
      pinMode(pin, PWM_MS_OUTPUT);
      RCLCPP_DEBUG(this->get_logger(), "Configured GPIO pin %d for PWM output", pin);
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

    // Initialise all pwm pins to 0
    for (const auto& pin : this->pwm_pins_) {
      pwmWrite(pin, 0); // Set initial PWM value to 0
      RCLCPP_DEBUG(this->get_logger(), "Set GPIO pin %d to initial PWM value 0", pin);
    }

    // Initialise the timer
    timer_ = nullptr;
  }

private:
  int get_max_duration(int brightness) {
    int duration = 500;
    if (brightness >= 100) {
      // RCLCPP_WARN(this->get_logger(), "Brightness is 100%%. "
      //   "This may cause damage to the LED. Use the force message field to override this check. "
      //   "Capping duration at %dms", duration);
      return duration;
    } else if (brightness > 50) {
      duration = -10 * brightness + 1500;
      // RCLCPP_WARN(this->get_logger(), "Brightness is at %d%%. "
      //   "This may cause damage to the LED. Use the force message field to override this check. "
      //   "Capping duration at %dms", brightness, duration);
      return duration;
    } else if (brightness > 30) {
      duration = -2950 * brightness + 148500;
      // RCLCPP_WARN(this->get_logger(), "Brightness is at %d%%. "
      //   "This may cause damage to the LED. Use the force message field to override this check. "
      //   "Capping duration at %dms", brightness, duration);
      return duration;
    } else {
      // do nothing
      // RCLCPP_DEBUG(this->get_logger(), "Brightness is at %d%%. "
      //   "This is a safe value for the LED.", brightness);
      return 0; // No maximum duration
    }
  }

  void led_control_callback(const VineLed::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: %s",
      to_string(msg).c_str());

    // Stop any existing timer
    if (timer_) {
      timer_->cancel();
      timer_ = nullptr;
    }

    int max_duration = get_max_duration(msg->brightness);
    if (!msg->force && msg->duration > max_duration && max_duration > 0) {
      // Check if the requested duration exceeds the maximum allowed duration
      // for the given brightness level
      RCLCPP_WARN(this->get_logger(),
                  "Requested duration %dms exceeds maximum allowed %dms for brightness %d%%. "
                  "Use force=true to override safety limits.",
                  msg->duration, max_duration, msg->brightness);
      return;
    }
    if (msg->force) {
      RCLCPP_WARN(this->get_logger(),
                  "Force mode enabled. Ignoring duration limits for brightness %d%%.",
                  msg->brightness);
    }


    this->brightness_ = msg->brightness;
    this->duration_ = msg->duration;

    // Set the PWM going
    configure_pwm(this->brightness_);

    // If duration is greater than 0, set a timer to turn off the LED
    if (this->duration_ > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Setting timer for %d milliseconds", this->duration_);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->duration_),
        [this]() {
          RCLCPP_DEBUG(this->get_logger(), "Duration (%d milliseconds) elapsed, turning off PWM",
            this->duration_);
          for (const auto& pin : this->pwm_pins_) {
            pwmWrite(pin, 0); // Turn off the LED
            RCLCPP_DEBUG(this->get_logger(), "Set GPIO pin %d to PWM value 0", pin);
          }
          // Reset the timer
          this->timer_ = nullptr;
          RCLCPP_DEBUG(this->get_logger(), "Timer reset");
        },
        nullptr
      );
    } else {
      // If duration is 0, turn off the LED immediately
      // for (const auto& pin : this->pwm_pins_) {
      //   pwmWrite(pin, 0); // Turn off the LED
      //   RCLCPP_INFO(this->get_logger(), "Set GPIO pin %d to PWM value 0", pin);
      // }
      // RCLCPP_INFO(this->get_logger(), "Duration is 0, LED turned off immediately");
      RCLCPP_INFO(this->get_logger(), "LED will remain on until a new command is received");
    }
  }

  void configure_pwm(const int brightness) {

    if (brightness < 0 || brightness > 100) {
      RCLCPP_ERROR(this->get_logger(), "Brightness value out of range: %d", brightness);
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Setting brightness to %d", brightness);

    // Set the PWM value based on the brightness level
    // Scale to 0-pwm_range and cap at 87.5% so we never go over 87.5% of 12V
    int pwm_value = static_cast<int>(((brightness * this->pwm_range) / 100) * 0.875);
    for (const auto& pin : this->pwm_pins_) {
      pwmWrite(pin, pwm_value);
      RCLCPP_DEBUG(this->get_logger(), "Set GPIO pin %d to PWM value %d", pin, pwm_value);
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

  rclcpp::Subscription<VineLed>::SharedPtr subscription_;
  std::vector<int> pwm_pins_; // Store the GPIO pins for PWM output
  int pwm_range; // Store the PWM range
  int pwm_clock; // Store the PWM clock

  int brightness_;
  int duration_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedControlVine>());
    rclcpp::shutdown();
    return 0;
}
