#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include <uvone_robot_msgs/LightCmd.h>
#include <std_srvs/Empty.h>
#include <sensor_door_msgs/CleanDisconnectedDevices.h>

#include <chrono>

struct Smooth {
    std::chrono::steady_clock::time_point last_time;
    double last_value {};

    const double acc {};

    explicit Smooth(double acc)
      : acc {acc}
    {}

    double call(double new_value)
    {
        std::chrono::steady_clock::time_point now { std::chrono::steady_clock::now() };
        const int64_t diff_us { std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count() };

        if (new_value>last_value)
        {
            last_value += acc*static_cast<double>(diff_us)/1000000.0f ;
            last_value = last_value > new_value ? new_value : last_value;
        }

        if (new_value<last_value)
        {
            last_value -= acc*static_cast<double>(diff_us)/1000000.0f ;
            last_value = last_value > new_value ? last_value : new_value;
        }   

        last_time = now;
        return last_value;
    }

    double zero()
    {
        last_value = 0;
        return last_value;
    }
};

struct VelocityDS4Control {
    static const double lin_acc_default; // m^2/s
    static const double ang_acc_default; // rad^2/s
    Smooth lin_smooth;
    Smooth ang_smooth;

    float lin_mult { 0.50f };
    float ang_mult { 1.60f };

    const ros::Publisher pub_vel;
    const ros::Publisher pub_feedback;

    geometry_msgs::Twist cmd_vel;
    ds4_driver::Status::ConstPtr last_msg;

    bool publish_cmd_vel {false};

    explicit VelocityDS4Control(ros::NodeHandle& nh)
      : lin_smooth { nh.param<double>("acceleration/linear/x", lin_acc_default) }
      , ang_smooth { nh.param<double>("acceleration/angular/z", ang_acc_default) }
      , pub_vel { nh.advertise<geometry_msgs::Twist>("cmd_vel", 400) }
      , pub_feedback { nh.advertise<ds4_driver::Feedback>("cmd_feedback", 400) }
    {}

    void update(const ds4_driver::Status::ConstPtr& msg)
    {
        update_multipliers(msg);
        update_mode(msg);        

        if (publish_cmd_vel || msg->button_circle >0)
            publish_vel(msg);

        last_msg = msg;
    }

    void update_mode(const ds4_driver::Status::ConstPtr& msg)
    {
        if (msg->button_r1 >0 && last_msg->button_r1 != msg->button_r1)
        {
            publish_cmd_vel = !publish_cmd_vel;
            ROS_INFO("Cambiando el modo de publicacion");
            ds4_driver::Feedback cmd_feedback;

            cmd_feedback.set_led = true;
            if (publish_cmd_vel)
            {
                cmd_feedback.led_r = 0.0f;
                cmd_feedback.led_g = 1.0f;
                cmd_feedback.led_b = 0.0f;
            }
            else
            {
                cmd_feedback.led_r = 1.0f;
                cmd_feedback.led_g = 0.0f;
                cmd_feedback.led_b = 0.0f;
            }
            cmd_feedback.set_rumble = true;
            cmd_feedback.rumble_duration = 0.15f;
            cmd_feedback.rumble_small = 0.1f;
            cmd_feedback.rumble_big = 0.2f;

            pub_feedback.publish(cmd_feedback);
        }
    }

    void publish_vel(const ds4_driver::Status::ConstPtr& msg)
    {
        const float target_linear_vel { (msg->axis_r2 - msg->axis_l2)*lin_mult };

        cmd_vel.linear.x = lin_smooth.call(target_linear_vel);
        if (cmd_vel.linear.x >= 0)
            cmd_vel.angular.z = ang_smooth.call(msg->axis_left_x*ang_mult);
        else
            cmd_vel.angular.z = ang_smooth.call(-(msg->axis_left_x*ang_mult));

        // Emergency stop
        if (msg->button_circle >0)
        {
            cmd_vel.linear.x = lin_smooth.zero();
            cmd_vel.angular.z = ang_smooth.zero();
        }

        pub_vel.publish(cmd_vel);
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
    }

};

const double VelocityDS4Control::lin_acc_default {0.4}; // m^2/s
const double VelocityDS4Control::ang_acc_default {3.0}; // rad^2/s

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

struct SystemControl {
    ros::ServiceClient srv_rearm;
    ros::ServiceClient srv_clean;
    ds4_driver::Status::ConstPtr last_msg;

    explicit SystemControl(ros::NodeHandle& nh)
      : srv_rearm( nh.serviceClient<std_srvs::Empty>("/sensor_door/security/rearm"))
      , srv_clean( nh.serviceClient<sensor_door_msgs::CleanDisconnectedDevices>("/sensor_door/door_server/clean_disconnected_devices"))
    {}

    void update(const ds4_driver::Status::ConstPtr& msg)
    {
        if (msg->button_square && msg->button_square != last_msg->button_square)
        {
            sensor_door_msgs::CleanDisconnectedDevices msg;
            srv_clean.call(msg);
            ROS_INFO("Calling clean_disconnected devices");
        }
        if (msg->button_triangle && msg->button_triangle != last_msg->button_triangle)
        {
            std_srvs::Empty msg;
            srv_rearm.call(msg);
            ROS_INFO("Calling rearm");
        }
        last_msg = msg;
    }
};

struct RemoteControl {
    VelocityDS4Control vel_control;
    LightControl light_control;
    SystemControl sys_control;

    const ros::Subscriber sub_ds4;

    explicit RemoteControl(ros::NodeHandle& nh)
      : vel_control { nh }
      , light_control { nh }
      , sys_control { nh }
      , sub_ds4 { nh.subscribe<ds4_driver::Status>("status", 400, &RemoteControl::callback, this) }
    {}

    void callback(const ds4_driver::Status::ConstPtr& msg) {

        if (msg->button_l1 && !msg->button_r1)
            light_control.update(msg);
        else
            vel_control.update(msg);
        sys_control.update(msg);
    }

};

int main(int argc, char** argv) {

    ros::init(argc, argv, "parser_ps4_vel");
    ros::NodeHandle nh("~");

    RemoteControl control{nh};

    ros::spin();    

    return 0;
}