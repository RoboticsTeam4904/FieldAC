#include <ntcore.h>
#include "remote.hpp"

Remote::Remote(std::string address, unsigned int port) {
    nt_inst = nt::GetDefaultInstance();
    nt::StartClient(nt_inst, address.c_str(), port);
    while(!nt::IsConnected(nt_inst))
        continue;
}

Remote::Remote(unsigned int team, unsigned int port) {
    nt_inst = nt::GetDefaultInstance();
    nt::StartClientTeam(nt_inst, team, port);
    while(!nt::IsConnected(nt_inst))
        continue;
}

void Remote::put_poses(std::string location, std::vector<Pose> poses) {
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> dists;
    std::vector<double> relangles;
    for(const Pose pose : poses) {
        xs.push_back(pose.x);
        ys.push_back(pose.y);
        dists.push_back(pose.dist);
        relangles.push_back(pose.relangle);
    }
    nt::SetEntryValue(location + "/x", nt::Value::MakeDoubleArray(xs));
    nt::SetEntryValue(location + "/y", nt::Value::MakeDoubleArray(ys));
    nt::SetEntryValue(location + "/dists", nt::Value::MakeDoubleArray(dists));
    nt::SetEntryValue(location + "/relangles", nt::Value::MakeDoubleArray(relangles));
}

SensorData Remote::get_sensordata() {
    SensorData sensorData;
    auto leftEncoder_table = nt::GetEntry(nt_inst, "/sensorData/leftEncoder");
    sensorData.leftEncoder = nt::GetEntryValue(leftEncoder_table)->GetDouble();
    auto rightEncoder_table = nt::GetEntry(nt_inst, "/sensorData/rightEncoder");
    sensorData.rightEncoder = nt::GetEntryValue(rightEncoder_table)->GetDouble();
    auto accelX_table = nt::GetEntry(nt_inst, "/sensorData/accelX");
    sensorData.accelX = nt::GetEntryValue(accelX_table)->GetDouble() * SensorData::IMU_TO_CM_S2;
    auto accelY_table = nt::GetEntry(nt_inst, "/sensorData/accelY");
    sensorData.accelY = nt::GetEntryValue(accelY_table)->GetDouble() * SensorData::IMU_TO_CM_S2;
    auto accelZ_table = nt::GetEntry(nt_inst, "/sensorData/accelZ");
    sensorData.accelZ = nt::GetEntryValue(accelZ_table)->GetDouble() * SensorData::IMU_TO_CM_S2;
    auto yaw = nt::GetEntry(nt_inst, "/sensorData/yaw");
    sensorData.yaw = (nt::GetEntryValue(yaw)->GetDouble()) * M_PI / 180;
    return sensorData;
}
