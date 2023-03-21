#pragma once
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "libackermann/libackermann.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class LogiG29Node : public rclcpp::Node {
    /// Joy node messages from wheel
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    /// Ackerman command outputs
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_pub;
    /// Twist command outputs
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

    /// Velocity we should give at full throttle
    float max_throttle_speed;
    /// Velocity we should give at full brake
    float max_braking_speed;
    /// Max steering angle allowed by this node (may be used to apply FF in the future)
    float max_steering_rad;
    /// Wheelbase of vehicle in meters
    float wheelbase;

    /// Phoenix's steering ratio
    const ack::Ratio steering_ratio = ack::get_steering_ratio(ack::Project::Phoenix);

public:
    LogiG29Node(const rclcpp::NodeOptions& options);

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
};