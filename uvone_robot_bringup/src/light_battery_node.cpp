#include <ros/ros.h>

#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Sound.h>

#include <chrono>

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

struct Battery_t {
    ros::Subscriber _voltage_sensor;
    SoundNode_t sound_;
    ros::Publisher _led2;

    std::chrono::time_point<std::chrono::system_clock> last_sound_call_;
    bool last_call_ { false };

    Battery_t(ros::NodeHandle& nh)
        : _voltage_sensor ( nh.subscribe("input_sensor", 1, &Battery_t::callback, this) )
        , sound_ ( nh )
        , _led2 ( nh.advertise<kobuki_msgs::Led>("led", 0, false) )
        , last_sound_call_ ( std::chrono::system_clock::now() )
    {}

    void callback(const kobuki_msgs::SensorStateConstPtr& msg) {
        const std::chrono::time_point<std::chrono::system_clock> now { std::chrono::system_clock::now() };
        auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sound_call_);

        static float avg_voltage {float(msg->analog_input[0])/float(1<<12)*3.3f*5.7f};
        constexpr int N {10};
        float voltage = float(msg->analog_input[0])/float(1<<12)*3.3f*5.7f;
        avg_voltage += -avg_voltage/N + voltage/N;

        if (time_between_sound(avg_voltage) != 0.0 && difference.count() >= time_between_sound(avg_voltage) ) {
            last_sound_call_ = now;
            sound_.send(kobuki_msgs::Sound::RECHARGE);
        }

        if ( avg_voltage < 10.0f && !last_call_ )
        {
            last_call_ = true;
            sound_.send(kobuki_msgs::Sound::OFF);
        }
        else if (avg_voltage > 11.0f && last_call_ )
        {
            last_call_ = false;
            sound_.send(kobuki_msgs::Sound::ON);
        }
        
        ROS_DEBUG("Voltaje de la bateria: %.2f V", avg_voltage);
        update_color(avg_voltage);
    }

    static float time_between_sound(float voltage)
    {
        float response { 0.0f };
        if (0.0f < voltage && voltage <= 12.0f)
            response =  5000.0f;
        else if (12.0f < voltage && voltage <= 13.0f)
            response = 30000.0f;
        else if (13.0f < voltage && voltage <= 14.5f)
            response = 60000.0f;
        return response;
    }

    void update_color(float voltage)
    {
        kobuki_msgs::Led led_msg {};
        if (voltage > 16.0f || voltage < 12.0f)
            led_msg.value = kobuki_msgs::Led::RED;
        else if (voltage < 14.0f)
            led_msg.value = kobuki_msgs::Led::ORANGE;
        else
            led_msg.value = kobuki_msgs::Led::GREEN;

        _led2.publish(led_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "light_battery_node");
    ros::NodeHandle nh;

    Battery_t light(nh);

    ROS_INFO("Node to measure batteries ready.");

    ros::spin();
}