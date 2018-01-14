#ifndef MCL_H
#define MCL_H

#include "../objects.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    void step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelYaw, std::string sensorData);
    void tick();
}

#endif
