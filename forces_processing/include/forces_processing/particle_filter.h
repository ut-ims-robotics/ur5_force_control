#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "forces_processing/particle_sample.h"

class ParticleFilter {
private:
    static const int PARTICLE_COUNT = 1000;
    std::vector <ParticleSample> particles;
    geometry_msgs::Vector3 current_measurement;
    float get_particle_weight(geometry_msgs::Vector3);
    float max_probability;
    void resample_particles();
public:

    ParticleFilter();
    void set_current_measurement(geometry_msgs::Vector3);
    void start();

};
