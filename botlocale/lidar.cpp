//
// Created by Howard Stark on 1/27/18.
//

#include "lidar.hpp"

lidarscan getAtLocation(int xCm, int yCm){

}
lidarscan generateExpected(const Pose& pose){
    return lidarscan(getAtLocation((int)pose.x,(int)pose.y), (int)(pose.yaw*180/3.14159));
}

