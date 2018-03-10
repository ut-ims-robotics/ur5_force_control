#include "forces_processing/ur5_contact_control_node.h"
#include "ur5_moveit/robot_mover.h"
#include "../../../ur5_path_planning/ur5_moveit/include/ur5_moveit/robot_mover.h"

void Ur5ContactControlNode::init() {
  ContactControl contactControl = ContactControl();
  contactControl.setFTTopic("wrench");
  contactControl.setVelTopic("jog_arm_server/delta_jog_cmds");
  contactControl.initialize("manipulator", "base_link", "base_link", "base_link", "base_link");

  ROS_INFO("Ur5ContactControlNode initialized successfully!");
  double maxForce = 50.0;
  double maxAllowedForce = 320.0;
  double speedCoef = 0.08;

  contactControl.setFollower(Contact::Dimension::DIM_X, maxForce, maxAllowedForce);
  //contactControl.setSpring(Contact::Dimension::DIM_X, 0.2, speedCoef, 1, maxAllowedForce);
  Contact::EndCondition endCondition = contactControl.move(maxAllowedForce, maxAllowedForce, speedCoef);

  ROS_INFO("EndCondition: %i", endCondition);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_contact_control_node");
  ros::NodeHandle nh;
  ros::Rate r(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();


  RobotMover robotMover = RobotMover();
  robotMover.moveToHome();
  ROS_INFO("Robot moved to home!");

  Ur5ContactControlNode ur5ContactControlNode = Ur5ContactControlNode();
  ur5ContactControlNode.init();

  return 0;
}
