#ifndef PROV_REMOTE_HPP
#define PROV_REMOTE_HPP


#include <string>

class DataFeed {
public:
    DataFeed(std::string port);
    void run();
private:
    std::string port;
};


#endif
