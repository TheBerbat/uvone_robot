#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <fstream>

std::ofstream file;

void callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("%.4f", msg->pose.position.x);
    file << "MOVE " << msg->pose.position.x << " " << msg->pose.position.y << " "<< msg->pose.orientation.z << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_writer");
    ros::NodeHandle nh{"~"};

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, callback);

    file.open("ok.txt");

    while(ros::ok())
        ros::spinOnce();

    file.close();

    return 0;
}