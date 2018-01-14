#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "particle_filter_node");
    ros::start();

    ros::Rate r(10); //10 hz

    ParticleFilter particleFilter;
    particleFilter.start();

    while (ros::ok()) {

        r.sleep();
    }
}
