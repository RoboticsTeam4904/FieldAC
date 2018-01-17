#include "field.hpp"
#include "vision.hpp"
#include "botlocale/mcl.hpp"
#include "objecttracking/cubetrack.hpp"
#include "bottracking/bottrack.hpp"

#include <sys/time.h>

void Field::tick() {
//	ObjectTracking::tick();
    timeval curTime;
    gettimeofday(&curTime, nullptr);
    int milli = curTime.tv_usec / 1000;

    char buffer [80];
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", buffer, milli);
    BotLocale::tick();
    BotTracking::tick();
}

/**
 * Singleton pattern for the field.
 * We hide the constructor so you can't instantiate a field outside the context
 * of the field, and then only create a Field upon the very first time `Field::getInstance` is called.
 */
Field::Field() = default;

Field* Field::instance = nullptr;
Field* Field::getInstance() {
    if(!instance) {
        instance = new Field();
    }
    return instance;
}
