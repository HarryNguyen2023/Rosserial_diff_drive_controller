#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "diff_drive_controller.h"

int main(int argc, char** argv)
{
    // Initiate of node
    ros::init(argc, argv, "diff_drive_node");
    ros::NodeHandle nh;

    // Allocate the class object
    DiffDriveController diff_drive_controller("robot/cmd_vel", 0.04, 0.3);

    // Declare the publishers
    diff_drive_controller.robot_vel_pub= nh.advertise<std_msgs::UInt32>(diff_drive_controller.robot_vel_topic, 1000);
    diff_drive_controller.robot_odom_pub = nh.advertise<nav_msgs::Odometry>(diff_drive_controller.robot_odom_topic, 1000);

    // Declare the subscriber
    diff_drive_controller.cmd_vel_sub = nh.subscribe(diff_drive_controller.cmd_vel_topic, 1000, &DiffDriveController::cmdVelCallBack, &diff_drive_controller);
    diff_drive_controller.robot_pos_sub = nh.subscribe(diff_drive_controller.robot_pos_topic, 1000, &DiffDriveController::robotPosCallBack, &diff_drive_controller);

    ros::spin();
    return 0;
}