#ifndef FIELD_H
#define FIELD_H

#include <vector>
#include "objects.hpp"

class Field {
private:
    static Field* instance;
    Field();
public:
    static Field* getInstance();
    void tick();
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose self;
};

#endif