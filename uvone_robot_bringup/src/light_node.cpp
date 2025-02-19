#include <ros/ros.h>

#include <kobuki_msgs/DigitalOutput.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/Led.h>

#include <uvone_robot_msgs/LightCmd.h>
#include <sensor_door_msgs/DoorSecurity.h>

class DigitalNode_t
{
    ros::Publisher digital_topic;
public:
    DigitalNode_t(ros::NodeHandle& nh)
    :   digital_topic(nh.advertise<kobuki_msgs::DigitalOutput>("digital_output", 0, true))
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
        RIGHT_LAMP  = false,
        LEFT_LAMP = true,
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

        selected_lamp = l;
        
        if (l == SelectorLamp::NONE)
            return;

        digital_node.send<static_cast<int>(SelectorFunction::LAMP_SELECTOR)>(static_cast<bool>(l));
        ros::Duration(0.1).sleep();
        
        if (state_inverter == StateInverter::ENABLE && l != SelectorLamp::NONE)
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
        if (state == StateInverter::ENABLE && selected_lamp != SelectorLamp::NONE)
            set_ballast(StateBallast::ENABLE);
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
      : sound_topic( nh.advertise<kobuki_msgs::Sound>("sound", 0, false) )
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

struct LedStatus_t {
    ros::Publisher pub_led;

    LedStatus_t(ros::NodeHandle& nh)
      : pub_led{ nh.advertise<kobuki_msgs::Led>("warning_led", 1000) }
    {}

    void red()
    {
        kobuki_msgs::Led msg;
        msg.value = kobuki_msgs::Led::RED;
        pub_led.publish(msg);
    }
    void orange()
    {
        kobuki_msgs::Led msg;
        msg.value = kobuki_msgs::Led::ORANGE;
        pub_led.publish(msg);
    }
    void green()
    {
        kobuki_msgs::Led msg;
        msg.value = kobuki_msgs::Led::GREEN;
        pub_led.publish(msg);
    }
    void black()
    {
        kobuki_msgs::Led msg;
        msg.value = kobuki_msgs::Led::BLACK;
        pub_led.publish(msg);
    }
};

struct LightNode_t {
    LampNode_t lamp_node;
    SoundNode_t sounds;
    ros::Subscriber cmds;
    ros::Subscriber door_status_sub;
    bool door_status = false;
    LedStatus_t led;

    LightNode_t(ros::NodeHandle& nh)
      : lamp_node( nh )
      , sounds( nh )
      , cmds( nh.subscribe("cmd_light", 10, &LightNode_t::callback, this) )
      , door_status_sub { nh.subscribe("sys_status", 10, &LightNode_t::callback_door_status, this) }
      , led { nh }
    {}

    void callback(const uvone_robot_msgs::LightCmd::ConstPtr& msg)
    {
        if (msg->inverter != 0)
        {
            if (door_status)
                lamp_node.set_inverter(LampNode_t::StateInverter::ENABLE);
            else
                ROS_INFO("Door status is disabled, cannot turn on the lights!");
        }
        else
            lamp_node.set_inverter(LampNode_t::StateInverter::DISABLE);

        switch (msg->lamp_selected)
        {
            case uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP:
                lamp_node.set_lamp(LampNode_t::SelectorLamp::NONE);
                break;
            case uvone_robot_msgs::LightCmd::SELECT_LEFT_LAMP:
                lamp_node.set_lamp(LampNode_t::SelectorLamp::LEFT_LAMP);
                break;
            case uvone_robot_msgs::LightCmd::SELECT_RIGHT_LAMP:
                lamp_node.set_lamp(LampNode_t::SelectorLamp::RIGHT_LAMP);
                break;
            default:
                ROS_WARN("Selected lamp with id: %d. Should be 0..2.", msg->lamp_selected);
                return;
        }

        if(msg->inverter)
        {
            if ( lamp_node.selected_lamp == LampNode_t::SelectorLamp::NONE )
            {
                sounds.set_period(1000);
                sounds.set_sound(3);
            }
            else
            {
                sounds.set_period(2000);
                sounds.set_sound(6);
            }
        }
        else
            sounds.silent();
    }

    void callback_door_status(const sensor_door_msgs::DoorSecurityConstPtr& msg)
    {
        if ( !msg->is_secure && door_status != msg->is_secure)
        {
            lamp_node.set_inverter(LampNode_t::StateInverter::DISABLE);
            led.red();
            sounds.silent();
            sounds.send(1);
        }
        else if (door_status != msg->is_secure)
        {
            sounds.send(0);
            led.green();
        }
        this->door_status = msg->is_secure;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "light_cmd_node");
    ros::NodeHandle nh("~");

    LightNode_t light(nh);

    ROS_INFO("Node to control lights ready.");

    ros::spin();
}
