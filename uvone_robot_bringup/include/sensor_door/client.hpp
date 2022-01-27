#pragma once

#include <mutex>
#include <netinet/in.h>
#include <atomic>

#include <thread>

#include "uvone_robot_bringup/DoorStatus.h"

namespace uvone_robot
{

class Client
{
private:
    std::mutex _conn_mtx;
        int _sock_fd;
        struct sockaddr_in _addr;
        std::string _id_device;

        std::atomic_bool _valid {false};        // Check if object is valid (not moved)
        std::atomic_bool _connected {false};
        std::atomic_bool _door_status {false};
        std::atomic_bool _last_response_successfull {false};

        std::atomic_bool _keep_theads;
        std::thread _update_thread;

public:
    static float refresh_rate;      // Hz (hay que sumarle aprox 10Hz respecto al real)
    static float timeout_ms;       // Tiempo de respuesta minimo

    Client();                                           // Constructor default
    Client(int sock_fd, struct sockaddr_in& _addr);     // Constructor
    ~Client() noexcept;                                 // Destructor
    Client(const Client& other) = delete;               // Copy constructor
    Client(Client&& other) noexcept;                    // Move constructor
    Client& operator=(const Client& other) = delete;    // Copy assignment
    Client& operator=(Client&& other) noexcept;         // Move assignment

    std::string read_msg();
    bool send_msg(const std::string& msg);

    std::string get_id() const { return _id_device; }
    bool is_connected() const { return _connected; }
    void disconnect();

    void start_thread() noexcept;
    void stop_thread() noexcept;
    void update_task();

    uvone_robot_bringup::DoorStatus get_ros_msg() const;
};


} // namespace uvone_robot
