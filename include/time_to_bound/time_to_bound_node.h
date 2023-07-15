/*Virtual fixtures for teleoperation based on space contraction
pd position control.
downscale deltas with tanh() when approaching box limits.
(optional: stiffness varies inversely.)

Author: Rafael Cabral
Date: 24.08.2022
*/

#pragma once

#include <pinocchio-interface/robot.h>

#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <ros/node_handle.h>

#include <time_to_bound/time_to_bound.h>

namespace time_to_bound {

class TimeToBoundNode {
public:
  TimeToBoundNode(ros::NodeHandle &node_handle);

private:

  TimeToBound* t2b;

  // robot kinematic model
  Robot robot; // load robot in main

  Eigen::Matrix<double, 6, 1> input_twist;

  // lower bound to obstacle | joint limit | min singular value
  double p_lb;
  double q_lb;
  double s_lb;

  double tau_lb;
  // // rename and extend (less conservative)
  // double t2b_lim_p;
  // double t2b_lim_q;
  // double t2b_lim_s;

  // franka emika robot joint limits
  Eigen::VectorXd q_min;
  Eigen::VectorXd q_max;  

  // box coordinates
  // init position 0.306619 5.17353e-05    0.487364 // do: cannot start if constraint active
  Eigen::Vector3d x_min;
  Eigen::Vector3d x_max;

  // admittance interface
  bool admittance_active;
  bool admittance_overwrite; // otherwise teleop input additive

  double admittance_force_thresh;
  double admittance_force2vel_factor;

  double admittance_torque_thresh;
  double admittance_torque2rot_factor;

  // end effector position
  Eigen::Vector3d x;

  Eigen::Matrix<double, 6, 1> O_F_ext;

  // joint configuration
  bool initq;

  void update();

  ros::Subscriber sub_external_wrench;
  void externalWrenchCallback(const geometry_msgs::WrenchStamped &msg);
  
  ros::Subscriber sub_joint_state;
  void jointStateCallback(const sensor_msgs::JointState &msg);

  // teleop vel subscriber 
  ros::Subscriber sub_teleop_twist;
  void teleopTwistCallback(const geometry_msgs::Twist &msg);

  // teleop vel publisher
  ros::Publisher pub_teleop_twist;

};

} // namespace time_to_bound
