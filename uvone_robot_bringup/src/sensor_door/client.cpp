#include "sensor_door/client.hpp"

#include <unistd.h>
#include <iostream>
#include <regex>

#include <ros/ros.h>

namespace uvone_robot
{

float Client::refresh_rate {30};      // Hz (hay que sumarle aprox 10Hz respecto al real)
float Client::timeout_ms {400};

Client::Client()
    : _sock_fd{-1}
    , _addr{}
    , _valid{false}
    , _id_device{""}
    , _keep_theads{false}
{
    //std::cout << "Default constructor\n";
}

Client::Client(int sock_fd, struct sockaddr_in& addr)
    : _sock_fd{sock_fd}
    , _addr{addr}
    , _valid{true}
    , _door_status{false}
    , _connected {true}
{
    static const std::regex id_device_regex 
        {"[A-F0-9]{2}:[A-F0-9]{2}:[A-F0-9]{2}:[A-F0-9]{2}:[A-F0-9]{2}:[A-F0-9]{2}"};
    _id_device = read_msg(); 
    if (!std::regex_match(_id_device, id_device_regex))
    {
        ROS_ERROR("Connected device with wrong id '%s'", _id_device.c_str());
        disconnect();
        //std::cout << _id_device.length();
        throw std::runtime_error("Not valid id");
    }
    //std::cout << "Device ID: '" << _id_device << "'\n";
    //std::cout << "Constructor\n";

    start_thread();
    
    ROS_INFO("New client is registered: '%s' (fd=%d)", _id_device.c_str(), _sock_fd);
}

Client::~Client() noexcept
{
    if (_valid)
    {
        try
        {
            disconnect();
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error client" << e.what() << '\n';
        }
    }

    //std::cout << "2Destructor " << _id_device << "\n";
    stop_thread();
}

Client::Client(Client&& other) noexcept
    : _sock_fd{other._sock_fd}
    , _addr{other._addr}
    , _id_device{other._id_device}
{
    other.stop_thread();
    stop_thread();
    if (_valid)
    {
        disconnect();
        ROS_WARN("Rewriting client registered with: '%s' (fd=%d) with another client registered with '%s' (fd=%d)", _id_device.c_str(), _sock_fd, other._id_device.c_str(), other._sock_fd);
    }
    _valid.store(other._valid);
    _door_status.store(other._door_status);
    _connected.store(other._connected);
    other._valid = false;
    //std::cout << "Move contructor\n";
    start_thread();
}

Client& Client::operator=(Client&& other) noexcept
{
    other.stop_thread();
    stop_thread();
    if (_valid)
    {
        disconnect();
        ROS_WARN("Rewriting client registered with: '%s' (fd=%d) with another client registered with '%s' (fd=%d)", _id_device.c_str(), _sock_fd, other._id_device.c_str(), other._sock_fd);
    }
    _sock_fd = other._sock_fd;
    _addr = other._addr;
    _id_device = other._id_device;
    
    _valid.store(other._valid);
    _door_status.store(other._door_status);
    _connected.store(other._connected);
    other._valid = false;
    //std::cout << "Move assignament\n";
    start_thread();

    return *this;
}

std::string Client::read_msg()
{
    if (!_valid)
        throw std::runtime_error("Can't read, client is disconnected");

    std::lock_guard<std::mutex> guard(_conn_mtx);
    constexpr const uint16_t buff_len {512};
    char buff[buff_len];
    const ssize_t len { recv(_sock_fd, buff, sizeof(buff)-1 , 0) };
    if (len < 0)
        throw std::runtime_error("Error during transmision");
    
    if (len == 0)
        throw std::runtime_error("Close transmision");

    buff[len] = 0;
    return std::string(buff);
}

bool Client::send_msg(const std::string& msg)
{
    if (!_valid)
        throw std::runtime_error("Can't send, client is disconnected");

    std::lock_guard<std::mutex> guard(_conn_mtx);
    const ssize_t bytes { write(_sock_fd, msg.c_str(), msg.length()) };
    return ( bytes >= static_cast<ssize_t>(msg.length()) );
}

void Client::disconnect()
{
    if (!_valid || !_connected)
        return;

    ROS_WARN("Disconnecting device '%s' (fd=%d)", _id_device.c_str(), _sock_fd);

    std::lock_guard<std::mutex> guard(_conn_mtx);
    char buff[256];

    if (0 != shutdown(_sock_fd, SHUT_RDWR));
    //    throw std::runtime_error("Cannot shutdown client");
    while (read(_sock_fd, buff, sizeof(buff)-1)>0);
    if (0 != close(_sock_fd));
    //    throw std::runtime_error("Cannot close client");

    _connected = false;
}


void Client::start_thread() noexcept
{
    if (!_keep_theads)
    {
        //std::cout << "Thread starting. ID: '" << _id <<"'\n";

        if (_update_thread.joinable())
            _update_thread.join();

        _keep_theads = true;
        _update_thread = std::thread(&Client::update_task, this);
        //std::cout << "Thread started successfully. ID: '" << _id_device <<"'\n";
    }
}

void Client::stop_thread() noexcept
{
    /*if (_keep_theads)
    {
        //std::cout << "Thread stopping. ID: '" << _id <<"'\n";
        _keep_theads = false;
        if (_update_thread.joinable())
            _update_thread.join();
        //std::cout << "Thread stopped successfully. ID: '" << _id_device <<"'\n";
    }*/
    
    _keep_theads = false;
    if (_update_thread.joinable())
        _update_thread.join();
}

void Client::update_task()
{
    int times_unsuccessfull{0};
    while (_keep_theads)
    {   
        try
        {
            const bool state { send_msg("STATUS?")};
            if (state)
            {
                std::string msg {read_msg()};
                
                times_unsuccessfull = 0;
                _last_response_successfull = true;

                if (msg == "STATUS?OK")
                {    
                    _door_status = true;
                }
                else
                {
                    _door_status = false;
                }
                
                //std::cout << " - " << _id_device << ": " << msg << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            ++times_unsuccessfull;
            _last_response_successfull = false;

            ROS_WARN("Timeout (%d) message device '%s' (fd=%d)", times_unsuccessfull, _id_device.c_str(), _sock_fd);
            if (times_unsuccessfull == 5)
            {
                disconnect();
                _keep_theads = false;
                ROS_ERROR("Device is disconnected '%s' (fd=%d). Stoping communication... ", _id_device.c_str(), _sock_fd);
            }
            //std::cerr << e.what() << '\n';
        }
    
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1000000.0/(refresh_rate))));
    }
}

uvone_robot_bringup::DoorStatus Client::get_ros_msg() const
{
    uvone_robot_bringup::DoorStatus msg;
    msg.door_id = _id_device;
    msg.last_response_successfull = _last_response_successfull;
    msg.connected = _connected;
    msg.door_status = _door_status;
    return msg;
}

} // namespace uvone_robot
