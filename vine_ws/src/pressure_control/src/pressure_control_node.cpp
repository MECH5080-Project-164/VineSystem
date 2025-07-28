#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;

class PressureControlNode : public rclcpp::Node {
public:
  PressureControlNode() : Node("pressure_control_node") {
    // Declare parameters with descriptions
    auto pressure_target_desc = rcl_interfaces::msg::ParameterDescriptor();
    pressure_target_desc.description = "Target pressure value in kPa";
    this->declare_parameter<double>("pressure_target", 0.0, pressure_target_desc);

    auto kp_desc = rcl_interfaces::msg::ParameterDescriptor();
    kp_desc.description = "Proportional gain for PID controller";
    this->declare_parameter<double>("kp", 1.0, kp_desc);

    auto ki_desc = rcl_interfaces::msg::ParameterDescriptor();
    ki_desc.description = "Integral gain for PID controller";
    this->declare_parameter<double>("ki", 0.1, ki_desc);

    auto kd_desc = rcl_interfaces::msg::ParameterDescriptor();
    kd_desc.description = "Derivative gain for PID controller";
    this->declare_parameter<double>("kd", 0.05, kd_desc);

    auto control_freq_desc = rcl_interfaces::msg::ParameterDescriptor();
    control_freq_desc.description = "Control loop frequency in Hz";
    this->declare_parameter<double>("control_frequency", 10.0, control_freq_desc);

    auto max_pwm_desc = rcl_interfaces::msg::ParameterDescriptor();
    max_pwm_desc.description = "Maximum PWM value (0-100)";
    this->declare_parameter<int>("max_pwm", 100, max_pwm_desc);

    auto min_pwm_desc = rcl_interfaces::msg::ParameterDescriptor();
    min_pwm_desc.description = "Minimum PWM value (0-100)";
    this->declare_parameter<int>("min_pwm", 0, min_pwm_desc);

    // Initialize variables
    current_pressure_ = 0.0;
    target_pressure_ = this->get_parameter("pressure_target").as_double();
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_pwm_ = this->get_parameter("max_pwm").as_int();
    min_pwm_ = this->get_parameter("min_pwm").as_int();

    integral_error_ = 0.0;
    previous_error_ = 0.0;
    last_time_ = this->now();

    // Create subscriber for pressure readings
    pressure_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "pressure", 10, std::bind(&PressureControlNode::pressure_callback, this, std::placeholders::_1));

    // Create publisher for PWM control
    pwm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("pump_pwm_control", 10);

    // Create timer for control loop
    double control_freq = this->get_parameter("control_frequency").as_double();
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq));
    control_timer_ = this->create_wall_timer(
      timer_period, std::bind(&PressureControlNode::control_loop, this));

    // Set up parameter callback for dynamic reconfiguration
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PressureControlNode::parameter_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Pressure Control Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: pressure");
    RCLCPP_INFO(this->get_logger(), "Publishing to: pump_pwm_control");
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.2f Hz", control_freq);
  }

private:
  void pressure_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_pressure_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received pressure: %.2f kPa", current_pressure_);
  }

  void control_loop() {
    // Get current time
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Only proceed if we have a valid time difference
    if (dt <= 0.0) {
      return;
    }

    // Calculate error
    double error = target_pressure_ - current_pressure_;

    // Calculate PID terms
    double proportional = kp_ * error;

    // integral_error_ += error * dt;
    // double integral = ki_ * integral_error_;

    // double derivative = 0.0;
    // if (dt > 0.0) {
    //   derivative = kd_ * (error - previous_error_) / dt;
    // }

    // Calculate total control output
    double control_output = proportional;

    // Convert to PWM value (0-100) and apply limits
    int pwm_value = static_cast<int>(control_output);
    pwm_value = std::max(min_pwm_, std::min(max_pwm_, pwm_value));

    // Publish PWM value
    auto pwm_msg = std_msgs::msg::Int32();
    pwm_msg.data = pwm_value;
    pwm_publisher_->publish(pwm_msg);

    // Log control information
    RCLCPP_DEBUG(this->get_logger(),
      "Control: Error=%.2f, P=%.2f, I=%.2f, D=%.2f, PWM=%d",
      error, proportional, integral, derivative, pwm_value);

    // Update for next iteration
    previous_error_ = error;
    last_time_ = current_time;
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "pressure_target") {
        target_pressure_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated target pressure to: %.2f kPa", target_pressure_);
      }
      else if (param.get_name() == "kp") {
        kp_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Kp to: %.3f", kp_);
      }
      else if (param.get_name() == "ki") {
        ki_ = param.as_double();
        integral_error_ = 0.0; // Reset integral when Ki changes
        RCLCPP_INFO(this->get_logger(), "Updated Ki to: %.3f", ki_);
      }
      else if (param.get_name() == "kd") {
        kd_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated Kd to: %.3f", kd_);
      }
      else if (param.get_name() == "max_pwm") {
        max_pwm_ = param.as_int();
        if (max_pwm_ < 0 || max_pwm_ > 100) {
          result.successful = false;
          result.reason = "max_pwm must be between 0 and 100";
          return result;
        }
        RCLCPP_INFO(this->get_logger(), "Updated max PWM to: %d", max_pwm_);
      }
      else if (param.get_name() == "min_pwm") {
        min_pwm_ = param.as_int();
        if (min_pwm_ < 0 || min_pwm_ > 100) {
          result.successful = false;
          result.reason = "min_pwm must be between 0 and 100";
          return result;
        }
        RCLCPP_INFO(this->get_logger(), "Updated min PWM to: %d", min_pwm_);
      }
      else if (param.get_name() == "control_frequency") {
        double new_freq = param.as_double();
        if (new_freq <= 0.0) {
          result.successful = false;
          result.reason = "control_frequency must be positive";
          return result;
        }
        // Recreate timer with new frequency
        control_timer_.reset();
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / new_freq));
        control_timer_ = this->create_wall_timer(
          timer_period, std::bind(&PressureControlNode::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "Updated control frequency to: %.2f Hz", new_freq);
      }
    }

    return result;
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pressure_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pwm_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Control variables
  double current_pressure_;
  double target_pressure_;
  double kp_, ki_, kd_;
  int max_pwm_, min_pwm_;

  // PID state variables
  double integral_error_;
  double previous_error_;
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureControlNode>());
  rclcpp::shutdown();
  return 0;
}
