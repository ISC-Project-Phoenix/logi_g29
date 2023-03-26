#include "logi_g29/logi_g29_node.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <functional>

#include "linux/input.h"

using namespace std::placeholders;

LogiG29Node::LogiG29Node(const rclcpp::NodeOptions& options) : Node("logi_g29", options) {
    this->joy_sub =
        this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&LogiG29Node::joy_callback, this, _1));
    this->ack_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 10);
    this->twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->max_braking_speed = this->declare_parameter<float>("max_braking_speed", -10.0);
    this->max_throttle_speed = this->declare_parameter<float>("max_throttle_speed", 10.0);
    this->max_steering_rad = this->declare_parameter<float>("max_steering_rad", 2.0);
    this->wheelbase = this->declare_parameter<float>("wheelbase", 1.8);

    this->force_feedback_fd =
        open("/dev/input/by-id/usb-Logitech_G29_Driving_Force_Racing_Wheel-event-joystick", O_RDWR);
}

/* Controller mapping:
 * axis 0: steering in 0-1 percent left is positive
 * quarter turn = .169
 * All pedals are -1.0 when not pressed, 1.0 full press, 0.0 half press
 * axis 1: clutch
 * axis 2: throttle
 * axis 3: brake
 *
 * buttons (1 indexed):
 * 13: first gear
 */

/// Maps pedals from -1 - 1 to 0 - 1
float normalize_pedal(float per) { return (per + 1) / 2; }

/// Maps steering from -1 - 1 to radians in ros convention
float steering2rad(float raw) { return 7.853981634f * raw; }

void LogiG29Node::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    // Steering here is in percents of one lock in either direction, could use for FF percents
    auto raw_steering = msg->axes[0];

    // Implement FF by just setting autocenter higher
    if (this->force_feedback_fd != -1) {
        auto level = std::clamp(abs(steering2rad(raw_steering)) / this->max_steering_rad, 0.2f, 1.0f);

        struct input_event ie;
        ie.type = EV_FF;
        ie.code = FF_AUTOCENTER;
        ie.value = 0xFFFFUL * level;

        if (write(this->force_feedback_fd, &ie, sizeof(ie)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write ff effect");
        }
    }

    auto raw_clutch = normalize_pedal(msg->axes[1]);
    auto raw_throttle = normalize_pedal(msg->axes[2]);
    auto raw_brake = normalize_pedal(msg->axes[3]);

    // Cap steering at max angle
    if (abs(steering2rad(raw_steering)) > this->max_steering_rad) {
        raw_steering = this->max_steering_rad * std::copysignf(1, raw_steering);
        RCLCPP_INFO(this->get_logger(), "Capping steering!");
    } else {
        raw_steering = steering2rad(raw_steering);
    }

    // First build an ack command, emulating the steering ratio
    auto raw_ack = AckermannCommand{0, this->steering_ratio(raw_steering)};

    // Prioritise throttle over brake
    if (raw_brake > 0) {
        raw_ack.speed = raw_brake * this->max_braking_speed;
    } else if (raw_throttle > 0) {
        raw_ack.speed = raw_throttle * this->max_throttle_speed;
    }

    // Send ackermann version of command
    {
        ackermann_msgs::msg::AckermannDrive ack_msg{};
        ack_msg.speed = raw_ack.speed;
        ack_msg.steering_angle = raw_ack.ackermann_angle;
        this->ack_pub->publish(ack_msg);
    }

    // Convert ackermann command to twist
    auto raw_tw = ack::ackermann_to_twist(raw_ack, this->wheelbase);
    geometry_msgs::msg::Twist twist_msg{};
    twist_msg.angular.z = raw_tw.v_angular_yaw;
    twist_msg.linear.x = raw_tw.v_linear_x;

    // Send twist version of command
    this->twist_pub->publish(twist_msg);
}
