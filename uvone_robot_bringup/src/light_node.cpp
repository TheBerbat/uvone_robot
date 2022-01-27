#include <ros/ros.h>
#include <kobuki_msgs/DigitalOutput.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/Led.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <csignal>

class DigitalNode_t
{
    ros::Publisher digital_topic;
public:
    DigitalNode_t(ros::NodeHandle& nh)
    :   digital_topic(nh.advertise<kobuki_msgs::DigitalOutput>("/mobile_base/commands/digital_output", 0, true))
    {}

    template <int DigitalOutput>
    bool send(bool state) const
    {
        if ( !has_connection() )
            return false;
        kobuki_msgs::DigitalOutput msg;
        msg.values.assign(state);
        msg.mask[DigitalOutput] = 1U;
        digital_topic.publish(msg);
        return true;
    }

    bool has_connection() const { return digital_topic.getNumSubscribers()>0; }
};

class LampNode_t {

public:
    enum class SelectorFunction {
        DIGITAL_INVERTER  = 1,
        DIGITAL_BALLAST   = 2,
        LAMP_SELECTOR     = 0
    };

    enum class SelectorLamp {
        LEFT_LAMP  = false,
        RIGHT_LAMP = true,
        NONE
    };

    enum class StateInverter {
        ENABLE   = false,
        DISABLE  = true,
    };

    enum class StateBallast {
        ENABLE    = false,
        DISABLE   = true,
    };

    const DigitalNode_t digital_node;
    SelectorLamp selected_lamp;
    StateInverter state_inverter;
    StateBallast state_ballast;

    LampNode_t(ros::NodeHandle& nh) 
     :  digital_node( nh )
    {
        while(!digital_node.has_connection());

        digital_node.send<static_cast<int>(SelectorFunction::DIGITAL_INVERTER)>(static_cast<bool>(StateInverter::DISABLE));
        state_inverter = StateInverter::DISABLE;

        digital_node.send<static_cast<int>(SelectorFunction::DIGITAL_BALLAST)>(static_cast<bool>(StateBallast::DISABLE));
        state_ballast = StateBallast::DISABLE;

        digital_node.send<static_cast<int>(SelectorFunction::LAMP_SELECTOR)>(static_cast<bool>(SelectorLamp::LEFT_LAMP));
        selected_lamp = SelectorLamp::NONE;
    }

    void set_lamp(SelectorLamp l)
    {
        if(selected_lamp == l)
            return;

        set_ballast(StateBallast::DISABLE);

        if (l == SelectorLamp::NONE)
            return;

        selected_lamp = l;
        digital_node.send<static_cast<int>(SelectorFunction::LAMP_SELECTOR)>(static_cast<bool>(l));
        ros::Duration(0.1).sleep();
        
        set_ballast(StateBallast::ENABLE);
    }

    void set_inverter(StateInverter state)
    {
        if (state_inverter == state)
            return;
        
        if (state == StateInverter::DISABLE)
            set_ballast(StateBallast::DISABLE);
        
        digital_node.send<static_cast<int>(SelectorFunction::DIGITAL_INVERTER)>(static_cast<bool>(state));
        state_inverter = state;
        ros::Duration(0.25).sleep();
    }

    void set_ballast(StateBallast state)
    {
        if (state_ballast == state)
            return;

        if (state == StateBallast::ENABLE)
            set_inverter(StateInverter::ENABLE);

        digital_node.send<static_cast<int>(SelectorFunction::DIGITAL_BALLAST)>(static_cast<bool>(state));
        state_ballast = state;
        ros::Duration(0.25).sleep();
    }
};

class SoundNode_t {
    using Sound = kobuki_msgs::Sound;

    ros::Publisher sound_topic;
    uint8_t actual_sound;
    ros::Timer timer;

public:
    SoundNode_t(ros::NodeHandle& nh)
      : sound_topic( nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 0, false) )
      , timer( nh.createTimer(ros::Rate(2), &SoundNode_t::remember_sound, this, false, false) )
    {}

    void remember_sound(const ros::TimerEvent& event) const 
    {
        send(actual_sound);
    }

    void set_sound(int id_sound)
    {
        actual_sound = id_sound;
        timer.start();
    }

    void set_period(int period_ms)
    {
        timer.setPeriod(ros::Duration(static_cast<double>(period_ms)/1000.0), false);
    }

    void send(uint8_t sound) const
    {
        kobuki_msgs::Sound msg;
        msg.value = sound;
        sound_topic.publish(msg);
    }

    void silent() { timer.stop(); }

};

struct LightNode_t {
    LampNode_t lamp_node;
    SoundNode_t sounds;


    ros::ServiceServer _select_light_r;
    ros::ServiceServer _select_light_l;
    ros::ServiceServer _power_off;
    ros::ServiceServer _enable_light;
    ros::ServiceServer _disable_light;
    ros::Subscriber _voltage_sensor;
    ros::Publisher _led2;

    LightNode_t(ros::NodeHandle& nh)
      : lamp_node( nh )
      , sounds( nh )
    {
        _select_light_r = nh.advertiseService("/select_light_right", &LightNode_t::callback_select_light_right, this);
        _select_light_l = nh.advertiseService("/select_light_left", &LightNode_t::callback_select_light_left, this);
        _power_off = nh.advertiseService("/select_light_none", &LightNode_t::callback_select_light_none, this);
        _enable_light = nh.advertiseService("/enable_light", &LightNode_t::callback_enable_light, this);
        _disable_light = nh.advertiseService("/disable_light", &LightNode_t::callback_disable_light, this);
        _voltage_sensor = nh.subscribe("/mobile_base/sensors/core", 1, &LightNode_t::get_analog_data, this);
        _led2 = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 0, false);

        //_sound_timer.stop();
    }

    void get_analog_data(const kobuki_msgs::SensorStateConstPtr& msg) {
        static float avg_voltage {float(msg->analog_input[0])/float(1<<12)*3.3f*5.7f};
        constexpr int N {10};
        float voltage = float(msg->analog_input[0])/float(1<<12)*3.3f*5.7f;
        avg_voltage -= avg_voltage/N;
        avg_voltage += voltage/N;
        ROS_DEBUG("Voltaje de la bateria: %.2f V", avg_voltage);

        kobuki_msgs::Led led_msg;

        if (avg_voltage > 16.0f || avg_voltage < 12.0f) {
            ROS_WARN("Voltaje de la bateria: %.2f V", avg_voltage);
            led_msg.value = led_msg.RED;
        } else if (avg_voltage < 14.0f) {
            ROS_WARN("Voltaje de la bateria: %.2f V", avg_voltage);
            led_msg.value = led_msg.ORANGE;
        } else {
            led_msg.value = led_msg.GREEN;
        }

        _led2.publish(led_msg);
    }

    bool callback_select_light_right(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        sounds.set_sound(1);
        sounds.set_period(2000);
        lamp_node.set_lamp(LampNode_t::SelectorLamp::LEFT_LAMP);
        return true;
    }

    bool callback_select_light_left(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        sounds.set_sound(1);
        sounds.set_period(2000);
        lamp_node.set_lamp(LampNode_t::SelectorLamp::RIGHT_LAMP);
        return true;
    }

    bool callback_select_light_none(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        sounds.set_sound(3);
        sounds.set_period(500);
        lamp_node.set_lamp(LampNode_t::SelectorLamp::NONE);
        return true;
    }

    bool callback_enable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        sounds.send(0);
        sounds.set_sound(3);
        sounds.set_period(500);

        lamp_node.set_inverter(LampNode_t::StateInverter::ENABLE);
        return true;
    }

    bool callback_disable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        sounds.send(1);
        sounds.silent();
        lamp_node.set_inverter(LampNode_t::StateInverter::DISABLE);
        return true;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "light_node");
    ros::NodeHandle nh;

    ROS_INFO("OK");

    LightNode_t light(nh);

    ros::spin();
}