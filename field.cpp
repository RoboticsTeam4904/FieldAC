#include "field.hpp"
#include "vision.hpp"
#include "botlocale/mcl.hpp"
#include "objecttracking/cubetrack.hpp"
#include "bottracking/bottrack.hpp"

#include <sys/time.h>

void Field::tick() {
	ObjectTracking::tick();
    cv::Mat* lol = Vision::getFrame();
    std::printf("Got frame");
    timeval curTime;
    gettimeofday(&curTime, nullptr);
    int milli = curTime.tv_usec / 1000;

    char buffer [80];
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", buffer, milli);
    printf("current time: %s \n", currentTime);
    BotLocale::tick();
    BotTracking::tick();
}