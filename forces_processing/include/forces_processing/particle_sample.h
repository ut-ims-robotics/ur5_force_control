#pragma once

#include <geometry_msgs/Vector3.h>

class ParticleSample {
private:

public:
    int count;
    ParticleSample() {
        count = 0;
    };
    geometry_msgs::Vector3 particle;
    float probability;
    float weight;
};
