#pragma once

#include <ros/ros.h>
#include <contact_control/contact_control.h>
#include <geometry_msgs/WrenchStamped.h>
#include "ur5_moveit/robot_mover.h"
#include "cartesian_impedance_msgs/ConfigureForceControl.h"

class Ur5ContactControlNode {
private:
    ros::NodeHandle nh_;
    ContactControl contact_control_ = ContactControl();
    std::vector<std::future<Contact::EndCondition>> pending_futures_;

    bool configure_cartesian_impedance(
        cartesian_impedance_msgs::ConfigureForceControl::Request &req,
        cartesian_impedance_msgs::ConfigureForceControl::Response &res);

public:
    Ur5ContactControlNode(ros::NodeHandle nh) {
      this->nh_ = nh;
    };

    void init();
};
