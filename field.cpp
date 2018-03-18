#include "field.hpp"

#include <sys/time.h>

Segment::Segment(int xi, int yi, int xf, int yf) {
    this->start = {xi, yi};
    this->end = {xf, yf};
}

Segment::Segment(std::tuple<int, int> start, std::tuple<int, int> end) {
    this->start = start;
    this->end = end;
}

Field::Field() = default;
Field* Field::Field::instance = nullptr;

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