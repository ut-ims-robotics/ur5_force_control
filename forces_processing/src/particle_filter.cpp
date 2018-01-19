#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"
#include <cmath>
#include <math.h>
#include <random>

ParticleFilter::ParticleFilter() {}

void ParticleFilter::update_measurement(geometry_msgs::Vector3 measurement) {
    current_measurement = measurement;
    move_particles();
    recalc_weights();
    resample_particles();
}

void ParticleFilter::move_particles() {
    std::default_random_engine generator;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        particles[i].count = 0;

        std::normal_distribution<double> distribution_x(particles[i].particle.x, 1);
        particles[i].particle.x = distribution_x(generator);
        std::normal_distribution<double> distribution_y(particles[i].particle.y, 1);
        particles[i].particle.y = distribution_y(generator);
        std::normal_distribution<double> distribution_z(particles[i].particle.z, 1);
        particles[i].particle.z = distribution_z(generator);
    }
}

float ParticleFilter::random_number(float min, float max)
{
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}


void ParticleFilter::start(geometry_msgs::Vector3 measurement) {
    ROS_INFO("## ParticleFilter started ##");

    current_measurement = measurement;

    srand(time(NULL));

    //Initialize particles
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        geometry_msgs::Vector3 vector;
        //TODO: maybe not fully at random but by the first measurement and gaussian distribution
        vector.x = random_number(-300, 300);
        vector.y = random_number(-300, 300);
        vector.z = random_number(-300, 300);
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
    float x_prob = gaussian_prob(current_measurement.x, 25, particle.x);
    float y_prob = gaussian_prob(current_measurement.y, 25, particle.y);
    float z_prob = gaussian_prob(current_measurement.z, 25, particle.z);

    return x_prob + y_prob + z_prob / 3;
}

float ParticleFilter::gaussian_prob(float mean, float variance, float value) {
    return exp(-pow((mean - value), 2) / pow(variance, 2) / 2.0) / sqrt(2.0 * M_PI * pow(variance, 2));
}

void ParticleFilter::resample_particles() {
    std::vector <ParticleSample> resampled_particles;
    int index = static_cast <int> (rand()) / (static_cast <int> (RAND_MAX / PARTICLE_COUNT));
    float beta = 0.0;
    int max_count = 0;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        float randomnr = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        beta = beta + randomnr * 2.0 * max_probability;

        while (beta > particles[index].probability) {
            beta = beta - particles[index].probability;
            index = (index + 1) % PARTICLE_COUNT;
            particles[index] = particles[index];
        }

        particles[index].count ++;
        if (particles[index].count > max_count) {
            max_count = particles[index].count;
            //most_probable_measurement = particles[index].particle;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;
}

geometry_msgs::Vector3 ParticleFilter::get_most_probable() {
    return most_probable_measurement;
}

