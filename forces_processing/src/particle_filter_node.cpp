#include "forces_processing/particle_filter_node.h"


void ParticleFilterNode::measurement_callback(const geometry_msgs::WrenchStamped &wrench_msg) {
    input_wrench_msg = wrench_msg;

    ROS_DEBUG("input force x: %f", input_wrench_msg.wrench.force.x);
    ROS_DEBUG("input force y: %f", input_wrench_msg.wrench.force.y);
    ROS_DEBUG("input force z: %f", input_wrench_msg.wrench.force.z);

    if (!initialized) {
        forcesParticleFilter = ParticleFilter();
        forcesParticleFilter.start(wrench_msg.wrench.force);

        torqueParticleFilter = ParticleFilter();
        torqueParticleFilter.start(wrench_msg.wrench.torque);
        initialized = true;
    }

    forcesParticleFilter.update_measurement(wrench_msg.wrench.force);
    torqueParticleFilter.update_measurement(wrench_msg.wrench.torque);

    publish_filtered_wrench();
}

void ParticleFilterNode::publish_filtered_wrench() {
    input_wrench_msg.wrench.force = forcesParticleFilter.get_most_probable();
    input_wrench_msg.wrench.torque = torqueParticleFilter.get_most_probable();

    ROS_DEBUG("filtered force x: %f", input_wrench_msg.wrench.force.x);
    ROS_DEBUG("filtered force y: %f", input_wrench_msg.wrench.force.y);
    ROS_DEBUG("filtered force z: %f", input_wrench_msg.wrench.force.z);
    ROS_DEBUG("===");

    ROS_DEBUG("filtered torque x: %f", input_wrench_msg.wrench.torque.x);
    ROS_DEBUG("filtered torque y: %f", input_wrench_msg.wrench.torque.y);
    ROS_DEBUG("filtered torque z: %f", input_wrench_msg.wrench.torque.z);
    ROS_DEBUG("===");

    filtered_value_pub.publish(input_wrench_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;
    ros::Rate r(10);

    ParticleFilterNode particleFilterNode = ParticleFilterNode(nh);

    geometry_msgs::Vector3 vector;
    srand(time(NULL));
    ros::spin();

    return 0;
}
