#include "control.h"

Control::Control() : rclcpp::Node("vehicle_control") {
    // Subscribe
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planner/path", 10, std::bind(&Control::path_callback, this,  std::placeholders::_1));
    velocity_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
        "/planner/velocity", 10, std::bind(&Control::velocity_callback, this,  std::placeholders::_1));
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/pose", 10, std::bind(&Control::pose_callback, this,  std::placeholders::_1));
    
    // Publish
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/controller/drive", 10);

    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Control::publisher_timer_callback, this)
    );
}

void Control::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    this->refPoses.clear();
    Pose refPose;
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        refPose.x = path_msg->poses[i].pose.position.x;
        refPose.y = path_msg->poses[i].pose.position.y;
        // refPose.heading = this->quat_to_yaw(path_msg->poses[i].pose.orientation);
        this->refPoses.push_back(refPose);
    }
}

void Control::velocity_callback(const std_msgs::msg::Int8::SharedPtr velocity_msg) {
    this->velocityValid = false;
    this->currVel = velocity_msg->data;
    if (this->currVel >= 0) {this->currDirection = true;}
    else {this->currDirection = false;}
    this->velocityValid = true;
}

void Control::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    this->poseValid = false;
    this->currPose.x = pose_msg->pose.position.x;
    this->currPose.y = pose_msg->pose.position.y;
    this->currPose.heading = this->quat_to_yaw(pose_msg->pose.orientation);
    this->poseValid = true;
}

void Control::publisher_timer_callback() {
    if (this->refPoses.empty() || !this->poseValid || !this->velocityValid) {return;}

    path_callback();
    velocity_callback();
    pose_callback();

    this->controller.purepursuit_control(this->currPose.x, this->currPose.y, this->currPose.heading, this->v);

    this->speedCommand = this->controller.getThrottle();
    this->steerCommand = - this->controller.getDelta() * 180 / M_PI;

    this->publish_drive(this->speedCommand, this->steerCommand);
}

void Control::publish_drive(float speed, float steer) {
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;

    drive_msg.header = std_msgs::msg::Header();
    drive_msg.header.stamp = rclcpp::Clock().now();
    drive_msg.header.frame_id = "map";

    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steer;

    this->drive_publisher_->publish(drive_msg);
}

float Control::quat_to_yaw(const geometry_msgs::msg::Quaternion quat) {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(tf2_quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}