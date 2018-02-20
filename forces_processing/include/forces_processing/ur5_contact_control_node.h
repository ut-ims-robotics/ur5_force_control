#pragma once

#include <ros/ros.h>
#include <contact_control/contact_control.h>
#include <geometry_msgs/WrenchStamped.h>

class Ur5ContactControlNode {
private:
  void init();
public:
    Ur5ContactControlNode() {};
};
