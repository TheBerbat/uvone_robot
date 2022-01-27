#include <ros/ros.h>
#include <kobuki_msgs/DigitalOutput.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/Led.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

struct LightNode_t {
    ros::Publisher _digital_output;
    ros::ServiceServer _select_light_r;
    ros::ServiceServer _select_light_l;
    ros::ServiceServer _power_off;
    ros::ServiceServer _enable_light;
    ros::ServiceServer _disable_light;
    ros::Subscriber _voltage_sensor;
    ros::Publisher _sounds;
    ros::Publisher _led2;
    ros::Timer _sound_timer;
    uint8_t _sound_selected;
    int32_t _selected_light {-1}; // 0->left  1->right  -1-> none
    bool _power_state;
    bool _inverter_state;
    bool _reading_state;

    LightNode_t(ros::NodeHandle* nh) {
        _digital_output = nh->advertise<kobuki_msgs::DigitalOutput>("/mobile_base/commands/digital_output", 10, true);
        _select_light_r = nh->advertiseService("/select_light_right", &LightNode_t::callback_select_light_right, this);
        _select_light_l = nh->advertiseService("/select_light_left", &LightNode_t::callback_select_light_left, this);
        _power_off = nh->advertiseService("/select_light_none", &LightNode_t::callback_select_light_none, this);
        _enable_light = nh->advertiseService("/enable_light", &LightNode_t::callback_enable_light, this);
        _disable_light = nh->advertiseService("/disable_light", &LightNode_t::callback_disable_light, this);
        _voltage_sensor = nh->subscribe("/mobile_base/sensors/core", 1, &LightNode_t::get_analog_data, this);
        _sounds = nh->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 0, false);
        _led2 = nh->advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 0, false);

        // Esperamos a que el nodo se haya suscrito al topic
        while(_digital_output.getNumSubscribers()<1);

        set_inverter(false);
        set_light(0);
        set_power(false);
        set_reading(true);
        _selected_light = -1;

        _sound_timer = nh->createTimer(ros::Rate(2), &LightNode_t::remember_sound, this, false, false);
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

    void set_light(uint32_t selected_light) {
        /* Si la luz ya está seleccionada, no hay nada que hacer */
        if(_selected_light == selected_light) {
            return;
        }

        /* Apagamos el inversor si está encendido, para evitar hacer el cambio con corriente */
        if (_power_state) {
            set_power(false);
            ros::Duration(0.25).sleep();
        }

        /* Actualizamos la variable que almacena que luz ha sido seleccionada */
        _selected_light = selected_light;

        /* Hcemos el cambio de seleccion del circuito de luz */
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {true, false, false, false};
        if (_selected_light == 1)
            msg.values = {true, false, false, false};
        else if (_selected_light == 0)
            msg.values = {false, false, false, false};
        _digital_output.publish(msg);
        
        /* Esperamos hasta que se haya hecho el cambio para encender el balastro */
        ros::Duration(0.25).sleep();
        set_power(true);
    }

    bool callback_select_light_right(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        /* Comprobamos si el robot esta preparado para encender las lamparas, si no devolvemos un error en el servicio */
        if (!_inverter_state)
            return false;

        _sound_selected = 1;
        _sound_timer.setPeriod(ros::Duration(2), false);
        set_light(0);
        return true;
    }

    bool callback_select_light_left(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        /* Comprobamos si el robot esta preparado para encender las lamparas, si no devolvemos un error en el servicio */
        if (!_inverter_state)
            return false;

        _sound_selected = 1;
        _sound_timer.setPeriod(ros::Duration(2), false);
        set_light(1);
        return true;
    }

    bool callback_select_light_none(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        /* Comprobamos si el robot esta preparado para encender las lamparas, si no devolvemos un error en el servicio */
        if (!_inverter_state)
            return false;

        /* Si ya esta apagado, no hace falta hacer mas */
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
            _selected_light = -1;
        }
        _sound_selected = 3;
        _sound_timer.setPeriod(ros::Duration(0.5));
        return true;
    }

    void set_power(bool value) {
        _power_state = value;
        update_ballast();
    }

    bool callback_enable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
        }
        set_inverter(true);
        send_sound(0);
        _sound_selected = 3;
        _sound_timer.setPeriod(ros::Duration(0.5));
        _sound_timer.start();

        return true;
    }
    bool callback_disable_light(std_srvs::Empty::Request &req, std_srvs::Empty::ResponseType &res) {
        if (_power_state) {
            set_power(0);
            ros::Duration(0.2).sleep();
        }
        set_inverter(false);
        send_sound(1);
        _sound_timer.stop();
        return true;
    }

    void set_inverter(bool value) {
        _inverter_state = value;
        if (value == 0) {
            _selected_light = -1;
        }
        update_inverter();
    }

    void set_reading(bool value) {
        _reading_state = value;
        update_reading();
    }

    void update_inverter() {
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {false, true, false, false};
        if (_inverter_state)
            msg.values = {false, false, false, false};
        else
            msg.values = {false, true, false, false};
        _digital_output.publish(msg);
    }

    void update_ballast() {
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {false, false, true, false};
        if (_power_state)
            msg.values = {false, false, false, false};
        else
            msg.values = {false, false, true, false};
        _digital_output.publish(msg);
    }

    void update_reading() {
        kobuki_msgs::DigitalOutput msg;
        msg.mask = {false, false, false, true};
        if (_reading_state == 1)
            msg.values = {false, false, false, false};
        else if (_reading_state == 0)
            msg.values = {false, false, false, true};
        _digital_output.publish(msg);
    }

    void send_sound(uint8_t id) {
        kobuki_msgs::Sound msg;
        msg.value = id;
        _sounds.publish(msg);
    }
    
    void remember_sound(const ros::TimerEvent& event) {
        send_sound(_sound_selected);
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "light_node");
    ros::NodeHandle nh;

    ROS_INFO("OK");

    LightNode_t light(&nh);

    ros::Rate r(500);
    int i = 0;
    while (ros::ok())
    {
        // TODO: reenviar mensajes para asegurar que nadie sobreescribe
        //light.update_inverter();
        //light.update_ballast();
        //light.update_selected_light();
        r.sleep();
        ros::spinOnce();
    }
    
    
    return 0;
}