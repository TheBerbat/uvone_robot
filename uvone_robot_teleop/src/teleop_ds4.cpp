#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include "uvone_robot_msgs/LightCmd.h"
#include <algorithm>

struct VelocityDS4Control {
    float lin_mult { 0.50f };
    float ang_mult { 1.10f };

    const ros::Publisher pub_vel;

    ds4_driver::Status::ConstPtr last_msg;

    explicit VelocityDS4Control(ros::NodeHandle& nh)
      : pub_vel { nh.advertise<geometry_msgs::Twist>("cmd_vel", 400) }
    {}

    void update(const ds4_driver::Status::ConstPtr& msg)
    {
        update_multipliers(msg);
        publish_vel(msg);
    }

    void publish_vel(const ds4_driver::Status::ConstPtr& msg) const
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = (msg->axis_r2 - msg->axis_l2)*lin_mult ;
        if (cmd.linear.x >= 0)
            cmd.angular.z = msg->axis_left_x*ang_mult;
        else
            cmd.angular.z = msg->axis_left_x*ang_mult;

        pub_vel.publish(cmd);
    }

    void update_multipliers(const ds4_driver::Status::ConstPtr& msg)
    {
        if ( msg->button_dpad_up && last_msg->button_dpad_up != msg->button_dpad_up) {
            lin_mult *= 1.05f;
            ROS_INFO("Aumentando velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_mult);
        } else if(msg->button_dpad_down && last_msg->button_dpad_down != msg->button_dpad_down) {
            lin_mult *= 0.95f;
            ROS_INFO("Reduciendo velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_mult);
        }

        if (msg->button_dpad_right && last_msg->button_dpad_right != msg->button_dpad_right) {
            ang_mult *= 1.05f;
            ROS_INFO("Aumentando velocidad angular un 5%%. ang_vel_mult=%.4f", ang_mult);
        } else if(msg->button_dpad_left && last_msg->button_dpad_left != msg->button_dpad_left) {
            ang_mult *= 0.95f;
            ROS_INFO("Reduciendo velocidad angular un 5%%. ang_vel_mult=%.4f", ang_mult);
        }

        last_msg = msg;
    }

};

struct LightControl {
    const ros::Publisher pub_light;
    uvone_robot_msgs::LightCmd pub_msg {};

    explicit LightControl(ros::NodeHandle& nh)
      : pub_light { nh.advertise<uvone_robot_msgs::LightCmd>("cmd_light", 400) }
    {}

    void update(const ds4_driver::Status::ConstPtr& msg)
    {
        if (msg->button_dpad_up)
            pub_msg.inverter = true;

        if (msg->button_dpad_down)
            pub_msg.inverter = false;

        if (msg->button_dpad_left)
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_LEFT_LAMP;

        if (msg->button_dpad_right)
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_RIGHT_LAMP;

        if (msg->button_options)
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP;

        pub_light.publish(pub_msg);
    }

};

struct RemoteControl {
    VelocityDS4Control vel_control;
    LightControl light_control;

    const ros::Subscriber sub_ds4;

    explicit RemoteControl(ros::NodeHandle& nh)
      : vel_control { nh }
      , light_control { nh }
      , sub_ds4 { nh.subscribe<ds4_driver::Status>("status", 400, &RemoteControl::callback, this) }
    {}

    void callback(const ds4_driver::Status::ConstPtr& msg) {

        if (msg->button_l1 && !msg->button_r1)
            light_control.update(msg);
        else
            vel_control.update(msg);

    }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "parser_ps4_vel");
    ros::NodeHandle nh("~");

    RemoteControl control{nh};

    ros::spin();    

    return 0;
}