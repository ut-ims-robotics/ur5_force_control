#pragma once

#include <geometry_msgs/Vector3.h>

class ParticleSample {
private:

public:
    ParticleSample() {};
    geometry_msgs::Vector3 particle;
    float probability;
    float weight;
};
