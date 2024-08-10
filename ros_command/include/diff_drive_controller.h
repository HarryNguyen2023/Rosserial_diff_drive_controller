#ifndef DIFF_DRIVER_CONTROLLER_H
#define DIFF_DRIVER_CONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <cmath>

using namespace std;

#define MAX_LEFT_VEL_RPM 250
#define MAX_RIGHT_VEL_RPM 250
#define PI 3.141618

union WordCombine
{
    int16_t robot_wheel_vel[2];
    uint32_t robot_vel_msg;
};

class DiffDriveController
{
private:
    float wheel_rad;
    float wheel_dist;

    uint16_t left_enc_ticks;
    uint16_t right_enc_ticks;

    float current_x;
    float current_y;
    float current_theta;

    nav_msgs::Odometry robot_odom;

    WordCombine send_buffer;
    WordCombine rcv_buffer;

    void diffDriveKinematics(const geometry_msgs::Twist& cmd_vel_msg, int16_t& left_vel_rpm, int16_t& right_vel_rpm);
    void robotPose(nav_msgs::Odometry& robot_odom, const int16_t& rev_left_enc, const int16_t& rev_right_enc);
public:
    std::string cmd_vel_topic;
    std::string robot_vel_topic;
    std::string robot_pos_topic;
    std::string robot_odom_topic;

    ros::Publisher robot_vel_pub;
    ros::Publisher robot_odom_pub;

    ros::Subscriber cmd_vel_sub;
    ros::Subscriber robot_pos_sub;

    DiffDriveController(std::string cmd_topic, float wheel_radius, float wheel_distance);
    void cmdVelCallBack(const geometry_msgs::Twist& cmd_vel_msg);
    void robotPosCallBack(const std_msgs::UInt32& robot_pos_msg);
};

// Class constructor
DiffDriveController::DiffDriveController(std::string cmd_topic, float wheel_radius, float wheel_distance)
{
    cmd_vel_topic = cmd_topic;
    robot_vel_topic = "robot_wheel_vel";
    robot_pos_topic = "robot_wheel_pos";
    robot_odom_topic = "robot/odom";

    this->left_enc_ticks = 374;
    this->right_enc_ticks = 374;

    this->wheel_rad = wheel_radius;
    this->wheel_dist = wheel_distance;

    current_x = 0;
    current_y = 0;
    current_theta = 0;
}

// Function to convert motor cmd_vel into wheel speeds in rpm
void DiffDriveController::diffDriveKinematics(const geometry_msgs::Twist& cmd_vel_msg, int16_t& left_vel_rpm, int16_t& right_vel_rpm)
{
    float left_vel, right_vel;
    left_vel = ((cmd_vel_msg.linear.x / wheel_rad) - (wheel_dist * cmd_vel_msg.angular.z) / (2.0 * wheel_rad)) * (60 / (2 * PI));
    right_vel = ((cmd_vel_msg.linear.x / wheel_rad) + (wheel_dist * cmd_vel_msg.angular.z) / (2.0 * wheel_rad)) * (60 / (2 * PI));

    // Constraint data
    if(left_vel > MAX_LEFT_VEL_RPM)
        left_vel = MAX_LEFT_VEL_RPM;
    else if(left_vel < - MAX_LEFT_VEL_RPM)
        left_vel = - MAX_LEFT_VEL_RPM;

    if(right_vel > MAX_RIGHT_VEL_RPM)
        right_vel = MAX_RIGHT_VEL_RPM;
    else if(right_vel < - MAX_RIGHT_VEL_RPM)
        right_vel = - MAX_RIGHT_VEL_RPM;

    left_vel_rpm = (int16_t)left_vel;
    right_vel_rpm = (int16_t)right_vel;
}

// Command velocity subscriber call back function
void DiffDriveController::cmdVelCallBack(const geometry_msgs::Twist& cmd_vel_msg)
{
    // Variables define
    int16_t left_vel_rpm;
    int16_t right_vel_rpm;
    std_msgs::UInt32 robot_wheel_vel;

    diffDriveKinematics(cmd_vel_msg, left_vel_rpm, right_vel_rpm);

    // Combine the wheel velocities into 1 msg
    send_buffer.robot_wheel_vel[0] = left_vel_rpm;
    send_buffer.robot_wheel_vel[1] = right_vel_rpm;
    robot_wheel_vel.data = send_buffer.robot_vel_msg;

    robot_vel_pub.publish(robot_wheel_vel);

    ROS_INFO("Left vel rpm: %d - Right vel rpm: %d - Send msg: %u\n", left_vel_rpm, right_vel_rpm, send_buffer.robot_vel_msg);
}

// Function to convertrelative encoder ticks into current robot pose
void DiffDriveController::robotPose(nav_msgs::Odometry& robot_odom, const int16_t& rev_left_enc, const int16_t& rev_right_enc)
{
    // Calculate the new relative wheel distances
    float left_distance = 2 * PI * wheel_rad * (rev_left_enc / (left_enc_ticks * 1.0));
    float right_distance = 2 * PI * wheel_rad * (rev_right_enc / (right_enc_ticks * 1.0));

    // Calculate the robot relatively new distances
    float delta_theta = (right_distance - left_distance) / wheel_dist;
    float delta_x = ((left_distance + right_distance) / 2.0) * std::cos(delta_theta);
    float delta_y = ((left_distance + right_distance) / 2.0) * std::sin(delta_theta);

    // ROS_INFO("Delta x: %.2f - Delta y: %.2f - Delta theta: %.2f\n", delta_x, delta_y, delta_theta);

    // Update robot current position
    current_theta += delta_theta;
    current_x += delta_x;
    current_y += delta_y;

    ROS_INFO("Current x: %.2f - Current y: %.2f - Current theta: %.2f\n", current_x, current_y, current_theta);

    // Update the Odometry msgs
    robot_odom.header.stamp = ros::Time::now();
    robot_odom.header.frame_id = "odom";
    robot_odom.pose.pose.position.x = current_x;
    robot_odom.pose.pose.position.y = current_y;
    robot_odom.pose.pose.orientation.z = sin(current_theta / 2.0);
    robot_odom.pose.pose.orientation.w = cos(current_theta / 2.0);
}

// Function for robot position callback
void DiffDriveController::robotPosCallBack(const std_msgs::UInt32& robot_pos_msg)
{
    int16_t rev_left_enc, rev_right_enc;

    // Convert message into robot relative encoder ticks
    rcv_buffer.robot_vel_msg = robot_pos_msg.data;
    rev_left_enc = rcv_buffer.robot_wheel_vel[0];
    rev_right_enc = rcv_buffer.robot_wheel_vel[1];

    robotPose(robot_odom, rev_left_enc, rev_right_enc);

    robot_odom_pub.publish(robot_odom);
}

#endif