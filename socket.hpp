#ifndef PROV_SOCKET_HPP
#define PROV_SOCKET_HPP

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>

class Socket {
private:
    int sock;
    struct sockaddr_in server;
public:
    std::string address;
    int port;
public:
    Socket(std::string address, int port);
    void open();
};

#endif
