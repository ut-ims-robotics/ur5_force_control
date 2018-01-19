#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "forces_processing/particle_sample.h"
#include <geometry_msgs/WrenchStamped.h>
#include <stdlib.h>
#include <time.h>

class ParticleFilter {
private:
    static const int PARTICLE_COUNT = 1000;
    std::vector <ParticleSample> particles;
    std::vector <std::pair <int,int>> index_to_count;
    geometry_msgs::Vector3 current_measurement;
    geometry_msgs::Vector3 most_probable_measurement;
    float get_particle_weight(geometry_msgs::Vector3);
    float gaussian_prob(float mean, float variance, float value);
    float max_probability;
    float random_number(float min, float max);
    void resample_particles();
    void recalc_weights();
    void move_particles();
public:
    ParticleFilter();
    void update_measurement(geometry_msgs::Vector3);
    void start(geometry_msgs::Vector3 measurement);
    geometry_msgs::Vector3 get_most_probable();
};
