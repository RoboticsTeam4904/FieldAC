#include "cubetrack.hpp"
#include "../field.hpp"

#include <opencv2/core.hpp>

namespace ObjectTracking {
    Cube::Cube(Pose pose) : pose(pose) {};

    /*
     * I did this not because I should
     * but because I knew I could
     */
    std::vector<Cube> Cube::findCubes() {
        std::vector<Cube> cubes;
        std::transform(Field::getInstance()->objects.begin(), Field::getInstance()->objects.end(),
                       cubes.begin(),
                       [](Pose p)->Cube{
                           return Cube(p);
                       }
        );
        return cubes;
    }

    void tick() {
        Cube::findCubes();
    }
}
