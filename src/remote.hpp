#ifndef PROV_REMOTE_HPP
#define PROV_REMOTE_HPP

#include <string>

class IRemote {
protected:
    std::string port;
public:
    IRemote(std::string port) : port(port) {};
    virtual void run() = 0;
};

class RemoteSocket : public IRemote {
public:
    RemoteSocket(const std::string &port);
    virtual void run();
};

#endif
