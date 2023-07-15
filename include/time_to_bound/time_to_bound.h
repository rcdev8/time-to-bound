/*Virtual fixtures for teleoperation based on space contraction
pd position control.
downscale deltas with tanh() when approaching box limits.
(optional: stiffness varies inversely.)

Author: Rafael Cabral
Date: 24.08.2022
*/

#pragma once

#include <Eigen/Dense>

namespace time_to_bound
{

  class TimeToBound
  {

  private:
    double getMinTau(Eigen::MatrixXd d_, Eigen::MatrixXd d_dot_);

  public:
    // lower bound to obstacle | joint limit | min singular value
    double p_lb;
    double q_lb;
    double s_lb;

    // rename and extend
    double t2b_min_p;
    double t2b_min_q;
    double t2b_min_s;

    // robot joint limits
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;

    // 3D box limits
    Eigen::Vector3d x_min;
    Eigen::Vector3d x_max;

    TimeToBound(double p_lb, double t2b_min_p, Eigen::Vector3d x_min, Eigen::Vector3d x_max, 
                        double q_lb, double t2b_min_q, Eigen::VectorXd q_min, Eigen::VectorXd q_max, 
                        double s_lb, double t2b_min_s);

    Eigen::VectorXd getSafeJointVelocity(const Eigen::Matrix<double, 6, 1> &twist_d,
                                const Eigen::Matrix<double, 4, 4> &O_T_EE, 
                                const Eigen::VectorXd &q,
                                const Eigen::VectorXd &dq,
                                const Eigen::MatrixXd &jacobian, 
                                const Eigen::MatrixXd &jacobianTimeVariation);
  };

} // namespace time_to_bound
