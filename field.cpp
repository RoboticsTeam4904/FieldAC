#include "field.hpp"

#include <sys/time.h>

Segment::Segment(int xi, int yi, int xf, int yf) : start({xi, yi}), end({xf, yf}) {}
Segment::Segment(std::array<int, 2> start, std::array<int, 2> end) : start(start), end(end) {}

Field::Field() = default;
Field::Field::instance = nullptr;

Field* Field::getInstance() {
    if(instance == nullptr) {
        instance = new Field();
    }
    return instance;
}

void Field::load() {
    /**
     *  TODO: Create a field from a vector graphic file-format
     *  TODO: using an std::vector of Segments
     */
}