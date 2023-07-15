// Time to bound safety for velocity based teleoperation
// Author: Rafael Cabral Date: 24.08.2022


#include <time_to_bound/time_to_bound_node.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>

namespace time_to_bound {

TimeToBoundNode::TimeToBoundNode(ros::NodeHandle &node_handle) {


  sub_teleop_twist = node_handle.subscribe(
      "/pre_teleop_twist_topic", 1, &TimeToBoundNode::teleopTwistCallback, this);

  pub_teleop_twist =
      node_handle.advertise<geometry_msgs::Twist>("/teleop_twist", 1);

  sub_joint_state = node_handle.subscribe(
    "/joint_states_topic", 1, &TimeToBoundNode::jointStateCallback, this);

  sub_external_wrench = node_handle.subscribe(
    "/external_wrench_topic", 1, &TimeToBoundNode::externalWrenchCallback, this);

  std::vector<double> x_max_;
  if (!node_handle.getParam("/time_to_bound/x_max", x_max_) ||
      x_max_.size() != 3) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no x_max parameters provided");
  }

  std::vector<double> x_min_;
  if (!node_handle.getParam("/time_to_bound/x_min", x_min_) ||
      x_min_.size() != 3) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no x_min parameters provided");
  }

  for (int i = 0; i < x_min.size(); i++) {
    x_min(i) = x_min_[i];
    x_max(i) = x_max_[i];
  }

  std::vector<double> q_max_;
  if (!node_handle.getParam("/time_to_bound/q_max", q_max_) ||
      q_max_.size() != 7) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no q_max parameters provided");
  }

  std::vector<double> q_min_;
  if (!node_handle.getParam("/time_to_bound/q_min", q_min_) ||
      q_min_.size() != 7) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no q_min parameters provided");
  }

  q_min.resize(q_min_.size());
  q_min.setZero();

  q_max.resize(q_max_.size());
  q_max.setZero();

  for (int i = 0; i < q_min.size(); i++) {
    q_min(i) = q_min_[i];
    q_max(i) = q_max_[i];
  }

  if (!node_handle.getParam("/time_to_bound/p_lb", p_lb)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no p_lb parameter provided");
  }

  if (!node_handle.getParam("/time_to_bound/q_lb", q_lb)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no q_lb parameter provided");
  }

  if (!node_handle.getParam("/time_to_bound/s_lb", s_lb)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no s_lb parameter provided");
  }

  if (!node_handle.getParam("/time_to_bound/tau_lb", tau_lb)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no tau_lb parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_active",
                            admittance_active)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_active "
              "parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_overwrite",
                            admittance_overwrite)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_overwrite "
              "parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_force_thresh",
                            admittance_force_thresh)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_force_thresh "
              "parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_force2vel_factor",
                            admittance_force2vel_factor)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_force2vel_factor "
              "parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_torque_thresh",
                            admittance_torque_thresh)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_torque_thresh "
              "parameter provided");
  }

  if (!node_handle.getParam("time_to_bound/admittance_torque2rot_factor",
                            admittance_torque2rot_factor)) {
    ROS_ERROR("TimeToBoundNode:  Invalid or no admittance_torque2rot_factor "
              "parameter provided");
  }


  // load pinocchio robot interface 
  std::string urdf_filename = ros::package::getPath("pinocchio-interface") + "/robots/panda_arm_hand_fixed_fingers_inertia.urdf";
  std::string frame_ee_name = "panda_hand_tcp";
  // initialize robot with an arbitrary valid configuration
  robot = Robot(urdf_filename, frame_ee_name, (q_min + q_max)/2, Eigen::VectorXd::Zero(q_min.size()));

  input_twist.setZero();
  O_F_ext.setZero();
  initq = false;

  
  ros::Rate rate(10);
  std::cout << "waiting for initial joint state" << std::endl;
  while (ros::ok() && !initq) { 
    ros::spinOnce();
  }
  std::cout << "received initial joint state" << std::endl;

  t2b = new TimeToBound(p_lb, tau_lb, x_min, x_max,
                q_lb, tau_lb, q_min, q_max,
                s_lb, tau_lb);

  // control loop
  update();
}



void TimeToBoundNode::teleopTwistCallback(const geometry_msgs::Twist &msg) {

  if (!initq){
    return;
  }

  input_twist << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
}

void TimeToBoundNode::jointStateCallback(const sensor_msgs::JointState &msg){
  Eigen::VectorXd q(robot.model.nv);
  Eigen::VectorXd dq(robot.model.nv);

  for (int i=0; i<robot.model.nv; i++){
    q(i) = msg.position[i];
    dq(i) = msg.velocity[i];
  }
  robot.updateKinematics(q, dq);
  
  if (!initq){
    initq = true;
  }

}

void TimeToBoundNode::externalWrenchCallback(const geometry_msgs::WrenchStamped &msg){
  Eigen::VectorXd EE_F_ext(6);
  EE_F_ext << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, 
                      msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  O_F_ext.setZero();
  O_F_ext.head(3) = robot.O_T_EE.block(0, 0, 3, 3)*EE_F_ext.head(3);
  O_F_ext.tail(3) = robot.O_T_EE.block(0, 0, 3, 3)*EE_F_ext.tail(3);

}

void TimeToBoundNode::update(){
  // control loop at rate param
  ros::Rate rate(100);
  while (ros::ok()) {

    Eigen::VectorXd admittance_twist(6);
    admittance_twist.setZero();

    Eigen::VectorXd safe_dq(robot.model.nv);
    safe_dq.setZero();

    // bool admittance_input = (O_F_ext.head(3).norm() > admittance_force_thresh) || (O_F_ext.tail(3).norm() > admittance_torque_thresh);
    bool admittance_input = (O_F_ext.head(3).norm() > admittance_force_thresh);
    
    if (admittance_input && admittance_active) {
      double force_scaling_factor = (O_F_ext.head(3).norm() - admittance_force_thresh) *
                              admittance_force2vel_factor / O_F_ext.head(3).norm();
      
      double torque_scaling_factor = (O_F_ext.tail(3).norm() - admittance_torque_thresh) *
                              admittance_torque2rot_factor / O_F_ext.tail(3).norm();
      
      Eigen::VectorXd scaling_factor(6);
      scaling_factor.setOnes();
      scaling_factor.head(3) *= force_scaling_factor;
      scaling_factor.tail(3) *= torque_scaling_factor;

      admittance_twist.array() = -scaling_factor.array() * O_F_ext.array();

      if (admittance_overwrite){
        // overwrite velocity command
        input_twist.setZero();
        safe_dq = t2b->getSafeJointVelocity(admittance_twist, robot.O_T_EE, robot.getq(), robot.getdq(), robot.J, robot.getdJ(robot.getq(), robot.getdq()));
      } else {
        // additive velocity command
        safe_dq = t2b->getSafeJointVelocity(admittance_twist + input_twist, robot.O_T_EE, robot.getq(), robot.getdq(), robot.J, robot.getdJ(robot.getq(), robot.getdq()));
      }

    } else {
      safe_dq = t2b->getSafeJointVelocity(input_twist, robot.O_T_EE, robot.getq(), robot.getdq(), robot.J, robot.getdJ(robot.getq(), robot.getdq()));
    }

    Eigen::VectorXd safe_twist = robot.J*safe_dq;

    std::cout << admittance_twist.transpose() << std::endl;
    std::cout << safe_twist.transpose() << std::endl << std::endl;

    // publishing
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = safe_twist(0);
    twist_msg.linear.y = safe_twist(1);
    twist_msg.linear.z = safe_twist(2);
    // twist_msg.angular.x = safe_twist(3);
    // twist_msg.angular.y = safe_twist(4);
    // twist_msg.angular.z = safe_twist(5);
    pub_teleop_twist.publish(twist_msg);

    ros::spinOnce();

    rate.sleep();
  }

  return;
}


} // namespace time_to_bound

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_to_bound");
  ros::NodeHandle nh;
  time_to_bound::TimeToBoundNode nc =
      time_to_bound::TimeToBoundNode(nh);
  ros::spin();
}