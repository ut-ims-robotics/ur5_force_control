#include "forces_processing/ur5_contact_control_node.h"



void Ur5ContactControlNode::init() {
  ContactControl contactControl = ContactControl();
  contactControl.setFTTopic("wrench");
  contactControl.setVelTopic("jog_arm_server/delta_jog_cmds");
  contactControl.initialize("manipulator", "base_link", "base_link", "base_link", "base_link");

  ROS_INFO("Ur5ContactControlNode initialized successfully!");

  contactControl.setFollower(Contact::Dimension::DIM_Y, 50, 50);


  Contact::EndCondition endCondition = contactControl.move(50.0, 100.0, 0.5);


  ROS_INFO("EndCondition: %i", endCondition);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_contact_control_node");
  ros::NodeHandle nh;
  ros::Rate r(10);

  Ur5ContactControlNode ur5ContactControlNode = Ur5ContactControlNode();
  ur5ContactControlNode.init();


  ros::spin();

  return 0;
}
