#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_vel_publisher;

void keyboardCallback(const std_msgs::String::ConstPtr& msg)
{
    
    ROS_INFO("Received message: %s", msg->data.c_str());

    geometry_msgs::Twist twist;

    std::string input = msg->data;
    if (input == "i") {  // Forward
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
    } else if (input == "u") {  // Left
        twist.linear.x = 0.5;
        twist.angular.z = 0.5;
    } else if (input == "o") {  // Right
        twist.linear.x = 0.5;
        twist.angular.z = -0.5;
    } else {  // Stop
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    cmd_vel_publisher.publish(twist);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "keyboard_controller");

    // Create node handle
    ros::NodeHandle node_handle;

    // Advertise the twist message publisher for controlling the Husky robot
    cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    // Subscribe to the keyboard input topic
    ros::Subscriber sub = node_handle.subscribe("/keyboard_reader/cmd", 10, keyboardCallback);

    ros::Rate rate(10); 

    while (ros::ok())
    {
        ros::spinOnce(); 
        rate.sleep();   
    }

    return 0;
}
