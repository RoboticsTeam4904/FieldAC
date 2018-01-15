#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../objects.hpp"

namespace ObjectTracking {
    class Cube {
    public:
        Cube(Pose pose);
        Pose pose;
        static std::vector<Cube> findCubes();
    };
    void tick();
}

#endif
