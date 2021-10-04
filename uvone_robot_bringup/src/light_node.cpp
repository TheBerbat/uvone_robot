#include <ros/ros.h>
#include <kobuki_msgs/DigitalOutput.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

struct LightNode_t {
    ros::Publisher _digital_output;
    ros::ServiceServer _select_light_r;
    ros::ServiceServer _select_light_l;
    ros::ServiceServer _power_off;
    ros::ServiceServer _enable_light;
    ros::ServiceServer _disable_light;
    uint32_t _selected_light; // 0->left  1->right
    bool _power_state;
    bool _inverter_state;

    LightNode_t(ros::NodeHandle* nh) {
        _digital_output = nh->advertise<kobuki_msgs::DigitalOutput>("/mobile_base/commands/digital_output", 10, true);
        _select_light_r = nh->advertiseService("/select_light_right", &LightNode_t::callback_select_light_right, this);
        _select_light_l = nh->advertiseService("/select_light_left", &LightNode_t::callback_select_light_left, this);
        _power_off = nh->advertiseService("/select_light_none", &LightNode_t::callback_select_light_none, this);
        _enable_light = nh->advertiseService("/enable_light", &LightNode_t::callback_enable_light, this);
        _disable_light = nh->advertiseService("/disable_light", &LightNode_t::callback_disable_light, this);

        while(_digital_output.getNumSubscribers()<1);

        set_inverter(false);
        set_light(false);
        set_power(false);
    }

    void set_light(uint32_t selected_light) {
        if (_power_state) {
            if(_selected_light == selected_light) {
                return;
            }
            set_power(false);
            ros::Duration(0.2).sleep();
        }

        _selected_light = selected_light;

        kobuki_msgs::DigitalOutput msg;
        msg.mask = {true, false, false, false};
        if (selected_light == 1)
            msg.values = {true, false, false, false};
        else if (selected_light == 0)
            msg.values = {false, false, false, false};
        _digital_output.publish(msg);

        ros::Duration(0.1).sleep();
        set_power(true);
    }

    bool callback_select_light_right(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (!_inverter_state)
            return false;
        set_light(0);
        return true;
    }
    bool callback_select_light_left(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (!_inverter_state)
            return false;
        set_light(1);
        return true;
    }
    bool callback_select_light_none(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (!_inverter_state)
            return false;
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
        }
        return true;
    }

    void set_power(bool value) {
        _power_state = value;
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {false, false, true, false};
        if (value)
            msg.values = {false, false, false, false};
        else
            msg.values = {false, false, true, false};
        _digital_output.publish(msg);
    }

    bool callback_enable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
        }
        for (int i=0; i<5; ++i) {
            set_inverter(false);
            ros::Duration(0.07).sleep();
            set_inverter(true);
            ros::Duration(0.15).sleep();
        }
        return true;
    }
    bool callback_disable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
        }
        set_inverter(false);
        return true;
    }

    void set_inverter(bool value) {
        _inverter_state = value;
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {false, true, false, false};
        if (value)
            msg.values = {false, false, false, false};
        else
            msg.values = {false, true, false, false};
        _digital_output.publish(msg);
    }
};




int main(int argc, char** argv) {
    ros::init(argc, argv, "light_node");
    ros::NodeHandle nh;

    ROS_INFO("OK");

    LightNode_t light(&nh);
    ros::Rate r(1);
    int i = 0;
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
    
    
    return 0;
}