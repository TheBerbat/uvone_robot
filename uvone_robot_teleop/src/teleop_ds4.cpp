#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include "uvone_robot_msgs/LightCmd.h"
#include <algorithm>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
void statusCallback(const ds4_driver::Status::ConstPtr& msg);
ros::Publisher cmd_vel;
float lin_vel_mult = 0.50;
float ang_vel_mult = 1.10;

ros::Publisher cmd_light;

struct VelocityDS4Control {
    float lin_mult { 0.50f };
    float ang_mult { 1.10f };

    const ros::Publisher pub_vel;

    uint32_t lb_up    {0};
    uint32_t lb_down  {0};
    uint32_t lb_right {0};
    uint32_t lb_left  {0};

    explicit VelocityDS4Control(ros::NodeHandle& nh)
      : pub_vel { nh.advertise<geometry_msgs::Twist>("cmd_vel", 400) }
    {}

    void update(const ds4_driver::Status::ConstPtr& msg)
    {
        update_multipliers(msg);
        publish_vel(msg);
    }

    void publish_vel(const ds4_driver::Status::ConstPtr& msg)
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = (msg->axis_r2 - msg->axis_l2)*lin_vel_mult ;
        if (cmd.linear.x >= 0)
            cmd.angular.z = msg->axis_left_x*ang_vel_mult;
        else
            cmd.angular.z = msg->axis_left_x*ang_vel_mult;

        pub_vel.publish(cmd);
    }

    void update_multipliers(const ds4_driver::Status::ConstPtr& msg)
    {
        if ( msg->button_dpad_up && lb_up != msg->button_dpad_up) {
            lin_vel_mult *= 1.05f;
            ROS_INFO("Aumentando velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_mult);
        } else if(msg->button_dpad_down && lb_down != msg->button_dpad_down) {
            lin_vel_mult *= 0.95f;
            ROS_INFO("Reduciendo velocidad lineal un 5%%. lin_vel_mult=%.4f", lin_mult);
        }
        lb_up = msg->button_dpad_up;
        lb_down = msg->button_dpad_down;

        if (msg->button_dpad_right && lb_right != msg->button_dpad_right) {
            ang_vel_mult *= 1.05f;
            ROS_INFO("Aumentando velocidad angular un 5%%. ang_vel_mult=%.4f", ang_vel_mult);
        } else if(msg->button_dpad_left && lb_left != msg->button_dpad_left) {
            ang_vel_mult *= 0.95f;
            ROS_INFO("Reduciendo velocidad angular un 5%%. ang_vel_mult=%.4f", ang_vel_mult);
        }
        lb_right = msg->button_dpad_right;
        lb_left = msg->button_dpad_left;
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
        if (msg->button_dpad_up) {
            pub_msg.inverter = true;
        }
        if (msg->button_dpad_down) {
            pub_msg.inverter = false;
        }
        if (msg->button_dpad_left) {
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_LEFT_LAMP;
        }
        if (msg->button_dpad_right) {
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_RIGHT_LAMP;
        }
        if (msg->button_options) {
            pub_msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP;
        }

        pub_light.publish(pub_msg);
    }

};

struct RemoteControl {
    //const ros::Publisher cmd_light;
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
        last_ang = std::min(target_ang, last_ang + 0.025f);
    } else if (last_ang > target_ang) {
        last_ang = std::max(target_ang, last_ang - 0.025f);
    }

    if (msg->button_circle == 1) {
        last_lin = 0;
        last_ang = 0;
    }

    msg_send.linear.x = last_lin;
    if (last_lin<0)
        msg_send.angular.z = -last_ang;
    else
    msg_send.angular.z = last_ang;
   

    cmd_vel.publish(msg_send);

    if (msg->button_l1) {
        static bool last_inverter = false;
        static uint8_t last_light = uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP;
        uvone_robot_msgs::LightCmd msg_light;
        // Ahora tratamos los mensajes de encendido de la lampara
        if (msg->button_dpad_up) {
            last_inverter = true;
        }
        if (msg->button_dpad_down) {
            last_inverter = false;
        }
        if (msg->button_dpad_left) {
            last_light = uvone_robot_msgs::LightCmd::SELECT_LEFT_LAMP;
        }
        if (msg->button_dpad_right) {
            last_light = uvone_robot_msgs::LightCmd::SELECT_RIGHT_LAMP;
        }
        if (msg->button_options) {
            last_light = uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP;
        }
        msg_light.inverter = last_inverter;
        msg_light.lamp_selected = last_light;

        cmd_light.publish(msg_light);
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "parser_ps4_vel");
    ros::NodeHandle nh("~");

    //cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //cmd_light = nh.advertise<uvone_robot_msgs::LightCmd>("cmd_light", 100);
    //ros::Subscriber ds4_cmd = nh.subscribe("/status", 1000, statusCallback);

    RemoteControl a{nh};

    ros::spin();    

    return 0;
}