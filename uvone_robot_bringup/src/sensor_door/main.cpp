#include <ros/ros.h>

#include "sensor_door/server.hpp"
#include "sensor_door/client.hpp"
#include "sensor_door/cluster.hpp"

#include <thread>
#include <unordered_map>

#include "uvone_robot_bringup/MultipleDoorStatus.h"
#include <std_srvs/SetBool.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sensor_door_server");
    ros::NodeHandle nh("sensor_door_server");

    int server_port {9999};
    nh.getParam("server_port", server_port);
    nh.getParam("client/refresh_rate", uvone_robot::Client::refresh_rate);
    nh.getParam("client/timeout_ms", uvone_robot::Client::timeout_ms);

    uvone_robot::Cluster cluster(server_port, 5);

    ros::Publisher pub = nh.advertise<uvone_robot_bringup::MultipleDoorStatus>("/door_status", 1000);
    ros::ServiceServer srv = nh.advertiseService("clean_disconnected_devices", &uvone_robot::Cluster::callback_clean_devices, &cluster);

    while(ros::ok())
    {
        auto msg {cluster.get_ros_msg()};
        pub.publish(msg);

        ros::spinOnce();
        ros::Rate(10).sleep();
    }
}