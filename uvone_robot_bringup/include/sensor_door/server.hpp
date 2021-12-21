#pragma once

#include <netinet/in.h>
#include "sensor_door/client.hpp"

namespace uvone_robot
{

class Server
{
private:
    constexpr static const uint32_t timeout_connect_ms {900}; // Must be less than 1000

    bool _valid {false}; // Check if object is valid (not moved)

    uint16_t _port;
    int _sock_fd;
    struct sockaddr_in _addr;
    uint8_t _queue;

    static int _create_raw_socket();
    static void _close_raw_socket(int sock_fd);


public:
    Server(uint16_t port, uint8_t queue = 5);           // Constructor
    ~Server() noexcept;                                 // Destructor
    Server(const Server& other) = delete;               // Copy constructor
    Server(Server&& other) noexcept;                    // Move constructor
    Server& operator=(const Server& other) = delete;    // Copy assignment
    Server& operator=(Server&& other) noexcept;         // Move assignment
    
    uint16_t get_port() const { return _port; }
    uint16_t get_sock() const { return _sock_fd; }   // TODO: Remove this dependency

    uvone_robot::Client accept_client() const;
};


} // namespace uvone_robot
