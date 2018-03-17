#include "forces_processing/ur5_contact_control_node.h"

void Ur5ContactControlNode::init() {
  ros::ServiceServer service = nh_.advertiseService("configure_cartesian_impedance",
                                                    &Ur5ContactControlNode::configure_cartesian_impedance, this);

  //Contact control initial configuration
  //TODO: make configurable
  contact_control_.setFTTopic("wrench");
  contact_control_.setVelTopic("jog_arm_server/delta_jog_cmds");
  contact_control_.initialize("manipulator", "base_link", "base_link", "base_link", "base_link");
  ROS_INFO("Ur5ContactControlNode initialized successfully!");

  while (ros::ok()) {
    ros::Duration(0.2).sleep();
    if (!pending_futures_.empty()) {
      pending_futures_.back().wait();
      Contact::EndCondition end_condition = pending_futures_.back().get();
      pending_futures_.pop_back();
      ROS_INFO("EndCondition: %i", end_condition);
    }
  }
}

bool Ur5ContactControlNode::configure_cartesian_impedance(
    cartesian_impedance_msgs::ConfigureForceControl::Request &req,
    cartesian_impedance_msgs::ConfigureForceControl::Response &res) {
  contact_control_.stopMove();
  ROS_INFO("Previous move stopped");

  std::map<std::string, Contact::Dimension> string_to_dimension;
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("X", Contact::Dimension::DIM_X));
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("Y", Contact::Dimension::DIM_Y));
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("Z", Contact::Dimension::DIM_Z));
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("RX", Contact::Dimension::DIM_RX));
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("RY", Contact::Dimension::DIM_RX));
  string_to_dimension.insert(std::pair<std::string, Contact::Dimension>("RZ", Contact::Dimension::DIM_RX));


  Contact::Dimension dimension = string_to_dimension.find(req.cart_force_control.DOF)->second;

  //TODO iterate over all dimensions
  double stiffness = req.cart_impedance_params.stiffness.translational.x;
  double damping = req.cart_impedance_params.damping.translational.x;
  double max_path_deviation = req.cart_impedance_params.max_path_deviation.translation.x;
  double max_ctrl_force = req.cart_impedance_params.max_ctrl_force.set.force.x;
  double max_ctrl_torque = req.cart_impedance_params.max_ctrl_force.set.torque.x;
  double max_ctrl_force_torque = std::min(max_ctrl_force, max_ctrl_torque);
  double max_cart_vel = req.cart_impedance_params.max_cart_vel.set.linear.x;


  contact_control_.setSpring(dimension, stiffness, damping, max_path_deviation, max_ctrl_force_torque);
  ROS_INFO(
      "Set spring rule with dimension: %s, "
          "stiffness: %f, damping: %f, "
          "max deviation: %f, max_ctrl_force_torque: %f", req.cart_force_control.DOF.c_str(), stiffness,
      damping,
      max_path_deviation,
      max_ctrl_force_torque);

  pending_futures_.push_back(contact_control_.moveAsync(max_ctrl_force, max_ctrl_torque, max_cart_vel));

  ROS_INFO("New move started");
  return true;
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

  Ur5ContactControlNode ur5ContactControlNode = Ur5ContactControlNode(nh);
  ur5ContactControlNode.init();

  return 0;
}
