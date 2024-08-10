#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

uint8_t count = 0;
bool up = true;

int main(int argc, char** argv)
{
    // Initiate ROS node
    ros::init(argc, argv, "cmd_vel_node");
    ros::NodeHandle nh;

    // Declare publisher
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("robot/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    // geometry_msgs::Twist cmd_vel_msg;
    geometry_msgs::Twist cmd_vel_msg;
    float linear_vel = 0;
    float angular_vel = 0;
    while(ros::ok())
    {
        // linear_vel = 0.7;
        // angular_vel = 0;
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;

        if(++count == 30)
        {
            count = 0;
            if(up)
            {
                linear_vel += 0.1;
                angular_vel += 0.1;
            }
            else
            {
                linear_vel -= 0.1;
                angular_vel -= 0.1;
            }
            
            if(linear_vel >= 1 || angular_vel >= 1)
                up = false;
            else if(linear_vel <= -1 || angular_vel <= -1)
                up = true;
        }
        ROS_INFO("Linear x: %.2f - Angular z: %.2f\n", linear_vel, angular_vel);
        // cmd_vel_pub.publish(cmd_vel_msg);
        cmd_vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}