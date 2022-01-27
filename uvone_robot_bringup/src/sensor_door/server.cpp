#include "sensor_door/server.hpp"

#include <ros/ros.h>
#include <fcntl.h>

namespace uvone_robot
{
    
Server::Server(uint16_t port, uint8_t queue)
    : _port {port}
    , _sock_fd{_create_raw_socket()}
    , _addr{.sin_family = AF_INET, .sin_port = htons(port), .sin_addr = {.s_addr = INADDR_ANY}, .sin_zero = {}}
    , _queue{queue}
{
    if (bind(_sock_fd, (struct sockaddr *)&_addr, sizeof(_addr))<0 || listen(_sock_fd, _queue) < 0)
    {
        if (0 != close(_sock_fd))
            throw std::runtime_error("Cannot close socket");
        throw std::runtime_error("Cannot binding or listening the socket");
    }
    else
        _valid = true;
}

Server::~Server() noexcept
{
    if(_valid)  // Only is valid (not moved)
    {
        try
        {
            _close_raw_socket(_sock_fd);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

Server::Server(Server&& other) noexcept
    : _port{other._port} 
    , _sock_fd{other._sock_fd}
    , _addr{other._addr}
    , _queue{other._queue}
    , _valid{other._valid}
{
    other._valid = false;
}

Server& Server::operator=(Server&& other) noexcept
{
    _port = other._port;
    _sock_fd = other._sock_fd;
    _addr = other._addr;
    _queue = other._queue;
    _valid = other._valid;

    other._valid = false;
    return *this;
}

int Server::_create_raw_socket()
{
    const int sock_fd{socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)};
    const int opt{1};
    
    if (sock_fd < 0)
        throw std::runtime_error("Could not create a socket");
    
    if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        throw std::runtime_error("Could not configure SO_REUSEADDR or SO_REUSEPORT socket server");

    // Does't work. IDK
    //int v { fcntl(sock_fd, F_GETFD, 0) };
    //fcntl(sock_fd, F_SETFD, v | O_NONBLOCK); // Disable socket block while accept. errno EWOULDBLOCK

    struct timeval timeout = { .tv_sec={}, .tv_usec=static_cast<uint32_t>(Server::timeout_connect_ms * 1000) };

    if (setsockopt (sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
        throw std::runtime_error("Could not configure SO_RCVTIMEO socket server");
    if (setsockopt (sock_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
        throw std::runtime_error("Could not configure SO_SNDTIMEO socket server");

    return sock_fd;
}

void Server::_close_raw_socket(int sock_fd)
{
    char buff[256];
    if (0 != shutdown(sock_fd, SHUT_RDWR))
        throw std::runtime_error("Cannot shutdown socket");
    while (read(sock_fd, buff, sizeof(buff)-1)>0);
    if (0 != close(sock_fd))
        throw std::runtime_error("Cannot close socket");
}

uvone_robot::Client Server::accept_client() const
{
    struct sockaddr_in client_addr {};
    socklen_t client_length { sizeof(client_addr) };

    int client_socket { accept(_sock_fd, (struct sockaddr*)&client_addr, &client_length) };
    if (client_socket < 0)
    {
        if (errno == EWOULDBLOCK)
            throw std::runtime_error("The queue is empty");
        else
            throw std::runtime_error("Error while accepting client");
    }

    const struct timeval timeout = { .tv_sec={}, .tv_usec=static_cast<uint32_t>(Client::timeout_ms * 1000) };

    if (setsockopt (client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
        throw std::runtime_error("Could not configure SO_RCVTIMEO socket client");
    if (setsockopt (client_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
        throw std::runtime_error("Could not configure SO_SNDTIMEO socket client");
    
    return {client_socket, client_addr};
}

} // namespace uvone_robot
