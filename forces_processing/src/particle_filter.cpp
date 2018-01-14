#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"


ParticleFilter::ParticleFilter() {
}

void ParticleFilter::set_current_measurement(geometry_msgs::Vector3 measurement) {
    current_measurement = measurement;
}

void ParticleFilter::start() {
    ROS_INFO("## ParticleFilter started ##");

    float weight_sum = 0.0;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        geometry_msgs::Vector3 vector;

        //TODO: maybe not fully at random but by the first measurement and gaussian distribution
        vector.x = ((float) rand() / (RAND_MAX));
        vector.y = ((float) rand() / (RAND_MAX));
        vector.z = ((float) rand() / (RAND_MAX));
        //

        ParticleSample sample = ParticleSample();
        sample.particle = vector;
        sample.weight = get_particle_weight(vector);
        weight_sum += sample.weight;
        particles.push_back(sample);

        //TODO: remove this logging
        ROS_INFO("vector.x: %f", sample.particle.x);
        ROS_INFO("vector.y: %f", sample.particle.y);
        ROS_INFO("vector.z: %f", sample.particle.z);
    }

    //Normalize the weights
    max_probability = 0.0;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        particles[i].probability = particles[i].weight / weight_sum;
        if (particles[i].probability > max_probability) {
            max_probability = particles[i].probability;
        }
    }

    resample_particles();


    ROS_INFO("Size: %d", (int) particles.size());
}



float ParticleFilter::get_particle_weight(geometry_msgs::Vector3) {
    return 0.0; //TODO: get weights depending on measurement
}

void ParticleFilter::resample_particles() {
    std::vector <ParticleSample> resampled_particles;
    int index = int(rand() * PARTICLE_COUNT); //TODO random value from 0 to PARTICLE_COUNT
    float beta = 0.0;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        beta += rand() * 2.0 * max_probability;
        ParticleSample current_particle = particles[index];
        while (beta > current_particle.probability) {
            beta -= current_particle.probability;
            index = (index + 1) % PARTICLE_COUNT;
            current_particle = particles[index];
        }

        //Reset weights
        current_particle.weight = 0.0;
        current_particle.probability = 0.0;
        //
        resampled_particles.push_back(current_particle);
    }

    particles = resampled_particles;
}

