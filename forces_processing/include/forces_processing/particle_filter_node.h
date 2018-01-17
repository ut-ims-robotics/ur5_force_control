#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "forces_processing/particle_sample.h"
#include <geometry_msgs/WrenchStamped.h>
#include "forces_processing/particle_filter.h"
#include "forces_processing/particle_sample.h"

class ParticleFilterNode {
private:
    void measurement_callback(const geometry_msgs::WrenchStamped &msg);

    ros::Publisher filtered_value_pub;
    ros::Subscriber measurement_sub;
    ros::NodeHandle nh_;
    bool initialized;
    ParticleFilter forcesParticleFilter;
    ParticleFilter torqueParticleFilter;
    geometry_msgs::WrenchStamped input_wrench_msg;
    void publish_filtered_wrench();
public:
    ParticleFilterNode(ros::NodeHandle &nh) {
        initialized = false;
        nh_ = nh;
        filtered_value_pub = nh.advertise<geometry_msgs::WrenchStamped>("/particle_filter/output", 1000);
        measurement_sub = nh.subscribe("/wrench", 1000, &ParticleFilterNode::measurement_callback, this);
    };
};
