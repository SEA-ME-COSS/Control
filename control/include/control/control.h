#pragma once

#include "msg_structs.h"

#include "controller/pure_pursuit.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "example_interfaces/msg/float64.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <cmath>

class Control : public rclcpp::Node {
public:
    Control();

private:
    PurePursuit controller;

    std::vector<Path> refPoses;
    Pose currPose;
    
    float speedCommand;
    float steerCommand;

    bool pathValid;
    bool poseValid;
    bool velocityValid;

    float target_velocity;

    // Subscribe
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr velocity_subscription_;

    // Publish
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void velocity_callback(const example_interfaces::msg::Float64::SharedPtr velocity_msg);
    void publisher_timer_callback();
    void publish_drive(float speed, float steer);
    float quat_to_yaw(const geometry_msgs::msg::Quaternion quat_msg);
};