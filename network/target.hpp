#ifndef INC_2018_FIELD_TARGET_HPP
#define INC_2018_FIELD_TARGET_HPP

struct Target {
    Target(float xCenter, float yCenter, float width, float height, float confidence);
    float xCenter;
    float yCenter;
    float width;
    float height;
    float confidence;

    friend bool operator== (const Target& t1, const Target& t2) {
        return (t1.xCenter == t2.xCenter) &&
                (t1.yCenter == t2.yCenter) &&
                (t1.width == t2.width) &&
                (t1.height == t2.height) &&
                (t1.confidence == t2.confidence);
    }
};

#endif