#pragma once
#include <unordered_map>
#include <thread>
#include <atomic>

#include "sensor_door/client.hpp"
#include "sensor_door/server.hpp"

#include "uvone_robot_bringup/MultipleDoorStatus.h"
#include "uvone_robot_bringup/CleanDisconnectedDevices.h"

namespace uvone_robot
{

class Cluster
{
private:
    Server _server;
    std::unordered_map<std::string, uvone_robot::Client> _clients;

    std::atomic_bool _keep_theads;
    std::thread _accept_client_thread;

    void _accept_client_task();
    
public:
    Cluster(uint16_t port, uint8_t queue);
    const std::unordered_map<std::string, uvone_robot::Client>& get_clients() const {return _clients;}
    ~Cluster();

    uvone_robot_bringup::MultipleDoorStatus get_ros_msg() const;
    bool callback_clean_devices(uvone_robot_bringup::CleanDisconnectedDevices::Request &req, uvone_robot_bringup::CleanDisconnectedDevices::Response &res);
};


} // namespace uvone_robot
