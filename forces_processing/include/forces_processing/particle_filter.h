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
    geometry_msgs::Vector3 most_probable_measurement;
    float get_particle_weight(geometry_msgs::Vector3);
    float gaussian_prob(float mean, float variance, float value);
    float max_probability;
    void resample_particles();
    void recalc_weights();
public:

    ParticleFilter();
    void update_measurement(geometry_msgs::Vector3);
    void start(geometry_msgs::Vector3 measurement);
    geometry_msgs::Vector3 get_most_probable();
};
