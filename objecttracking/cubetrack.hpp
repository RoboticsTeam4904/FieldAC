#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../objects.hpp"

namespace ObjectTracking {
    class Cube {
    public:
        Pose pose;
    };
    std::vector<Cube> findCubes();
    void tick();
}

#endif
