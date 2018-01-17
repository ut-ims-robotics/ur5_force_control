#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "particle_filter_node");
    ros::start();

    ros::Rate r(10); //10 hz


    //Initial mock vector JUST FOR TESTING AT THE MOMENT
    srand(time(0));

    geometry_msgs::Vector3 mock_vector;
    mock_vector.x = ((float) rand() / (RAND_MAX));
    mock_vector.y = ((float) rand() / (RAND_MAX));
    mock_vector.z = ((float) rand() / (RAND_MAX));


    ParticleFilter particleFilter;
    particleFilter.start(mock_vector);

    geometry_msgs::Vector3 filtered = particleFilter.get_most_probable();
    ROS_DEBUG("initial x: %f", mock_vector.x);
    ROS_DEBUG("initial y: %f", mock_vector.y);
    ROS_DEBUG("initial z: %f", mock_vector.z);
    ROS_DEBUG("===");
    ROS_DEBUG("filtered x: %f", filtered.x);
    ROS_DEBUG("filtered y: %f", filtered.y);
    ROS_DEBUG("filtered z: %f", filtered.z);
    ROS_DEBUG("===");

    particleFilter.update_measurement(mock_vector);
    geometry_msgs::Vector3 filtered2 = particleFilter.get_most_probable();
    ROS_DEBUG("initial x: %f", mock_vector.x);
    ROS_DEBUG("initial y: %f", mock_vector.y);
    ROS_DEBUG("initial z: %f", mock_vector.z);
    ROS_DEBUG("===");
    ROS_DEBUG("filtered2 x: %f", filtered2.x);
    ROS_DEBUG("filtered2 y: %f", filtered2.y);
    ROS_DEBUG("filtered2 z: %f", filtered2.z);


//    while (ros::ok()) {
//
//        r.sleep();
//    }
}
