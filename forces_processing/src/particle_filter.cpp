#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"
#include <cmath>
#include <math.h>


ParticleFilter::ParticleFilter() {
}

void ParticleFilter::update_measurement(geometry_msgs::Vector3 measurement) {
    current_measurement = measurement;
    //Move particles (cannot move because there is no motion model for force sensing)
    recalc_weights();
    resample_particles();
}


void ParticleFilter::start(geometry_msgs::Vector3 measurement) {
    ROS_INFO("## ParticleFilter started ##");

    current_measurement = measurement;

    //srand(time(1));

    //Initialize particles
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        geometry_msgs::Vector3 vector;
        //TODO: maybe not fully at random but by the first measurement and gaussian distribution
        vector.x = ((float) rand() / (RAND_MAX)); //TODO determine the range
        vector.y = ((float) rand() / (RAND_MAX));
        vector.z = ((float) rand() / (RAND_MAX));
        //

        ROS_DEBUG("rand x: %f", vector.x);
        ROS_DEBUG("rand y: %f", vector.y);
        ROS_DEBUG("rand z: %f", vector.z);
        ROS_DEBUG("===");

        ParticleSample sample = ParticleSample();
        sample.particle = vector;
        particles.push_back(sample);
    }


    recalc_weights();
    resample_particles();
}


void ParticleFilter::recalc_weights() {
    float weight_sum = 0.0;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        //TODO maybe consider last step's weight and get mean of the two
        particles[i].weight = get_particle_weight(particles[i].particle);
        weight_sum += particles[i].weight;
    }


    //Normalize the weights
    max_probability = 0.0;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        particles[i].probability = particles[i].weight / weight_sum;
        if (particles[i].probability > max_probability) {
            max_probability = particles[i].probability;
            most_probable_measurement = particles[i].particle;
        }
    }
}


float ParticleFilter::get_particle_weight(geometry_msgs::Vector3 particle) {
    float x_prob = gaussian_prob(current_measurement.x, 0.05, particle.x);
    float y_prob = gaussian_prob(current_measurement.y, 0.05, particle.y);
    float z_prob = gaussian_prob(current_measurement.z, 0.05, particle.z);

    return x_prob + y_prob + z_prob / 3;
}

float ParticleFilter::gaussian_prob(float mean, float variance, float value) {
    return exp(-pow((mean - value), 2) / pow(variance, 2) / 2.0) / sqrt(2.0 * M_PI * pow(variance, 2));
}

void ParticleFilter::resample_particles() {
    std::vector <ParticleSample> resampled_particles;
    int index = static_cast <int> (rand()) / (static_cast <int> (RAND_MAX / PARTICLE_COUNT));
    float beta = 0.0;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        float randomnr = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        beta = beta + randomnr * 2.0 * max_probability;

        while (beta > particles[index].probability) {
            beta = beta - particles[index].probability;
            index = (index + 1) % PARTICLE_COUNT;
            particles[index] = particles[index];
        }

        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;
}

geometry_msgs::Vector3 ParticleFilter::get_most_probable() {
    return most_probable_measurement;
}

