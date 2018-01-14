#include "field.h"
#include "botlocale/mcl.h"
#include "objecttracking/cubetrack.h"
#include "vision.h"
#include "bottracking/bottrack.h"

void Field::tick() {
	ObjectTracking::tick();
    cv::Mat* lol = Vision::getFrame();
    std::printf("Got frame");
    std::printf("%d\n", std::chrono::system_clock::now());
    BotLocale::tick();
    BotTracking::tick();
}