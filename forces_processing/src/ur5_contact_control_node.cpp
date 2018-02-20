#include "forces_processing/ur5_contact_control_node.h"



void Ur5ContactControlNode::init() {
  ContactControl contactControl = ContactControl();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_contact_control_node");
  ros::NodeHandle nh;
  ros::Rate r(10);

  Ur5ContactControlNode ur5ContactControlNode = Ur5ContactControlNode();

  ros::spin();

  return 0;
}
