#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include <algorithm>

ros::Publisher cmd_vel;
float lin_vel_mult = 0.15;
float ang_vel_mult = 0.30;


void statusCallback(const ds4_driver::Status::ConstPtr& msg) {
    static int last_button_dpad_up = 0;
    static int last_button_dpad_down = 0;
    static int last_button_dpad_right = 0;
    static int last_button_dpad_left = 0;
    float target_lin, target_ang;
    static float last_lin = 0, last_ang = 0;
    geometry_msgs::Twist msg_send;

    if (msg->button_dpad_up && last_button_dpad_up != msg->button_dpad_up) {
        lin_vel_mult *= 1.05f;
        ROS_INFO("Aumentando velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_vel_mult);
    } else if(msg->button_dpad_down && last_button_dpad_down != msg->button_dpad_down) {
        lin_vel_mult *= 0.95f;
        ROS_INFO("Reduciendo velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_vel_mult);
    }
    target_lin = (msg->axis_r2 - msg->axis_l2)*lin_vel_mult;
    last_button_dpad_up = msg->button_dpad_up;
    last_button_dpad_down = msg->button_dpad_down;

    if (msg->button_dpad_right && last_button_dpad_right != msg->button_dpad_right) {
        ang_vel_mult *= 1.05f;
        ROS_INFO("Aumentando velocidad angular un 5%%. ang_vel_mult=%.4f", ang_vel_mult);
    } else if(msg->button_dpad_left && last_button_dpad_left != msg->button_dpad_left) {
        ang_vel_mult *= 0.95f;
        ROS_INFO("Reduciendo velocidad angular un 5%%. ang_vel_mult=%.4f", ang_vel_mult);
    }
    target_ang = msg->axis_left_x*ang_vel_mult;
    last_button_dpad_right = msg->button_dpad_right;
    last_button_dpad_left = msg->button_dpad_left;
    
    if(last_lin < target_lin) {
        last_lin = std::min(target_lin, last_lin + 0.004f);
    } else if (last_lin > target_lin) {
        last_lin = std::max(target_lin, last_lin - 0.004f);
    }

    if(last_ang < target_ang) {
        last_ang = std::min(target_ang, last_ang + 0.002f);
    } else if (last_ang > target_ang) {
        last_ang = std::max(target_ang, last_ang - 0.002f);
    }

    if (msg->button_circle == 1) {
        last_lin = 0;
        last_ang = 0;
    }

    msg_send.linear.x = last_lin;
    msg_send.angular.z = last_ang;

    cmd_vel.publish(msg_send);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "parser_ps4_vel");
    ros::NodeHandle nh;

    cmd_vel = nh.advertise<geometry_msgs::Twist>("/uvone_teleop_keyboard/cmd_vel", 1000);
    ros::Subscriber ds4_cmd = nh.subscribe("/status", 1000, statusCallback);

    ros::spin();    

    return 0;
}