#pragma once

#include "msg_structs.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <cmath>

class Control : public rclcpp::Node {
public:
    Control();

private:
    std::vector<Pose> refPoses;
    Pose currPose;
    int currVel;
    bool currDirection;
    
    float speedCommand;
    float steerCommand;

    bool velocityValid;
    bool poseValid;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void velocity_callback(const std_msgs::msg::Int8::SharedPtr velocity_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    // void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void publisher_timer_callback();
    void publish_drive(float speed, float steer);
    float quat_to_yaw(const geometry_msgs::msg::Quaternion quat_msg);
};