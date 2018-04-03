#ifndef PROV_REMOTE_HPP
#define PROV_REMOTE_HPP


#include "objects.hpp"

class Remote {
private:
    NT_Inst nt_inst;
public:
    Remote(std::string address, unsigned int port);
    Remote(unsigned int team, unsigned int port);

    void put_poses(std::string location, std::vector<Pose>);
    SensorData get_sensordata();
};


#endif
