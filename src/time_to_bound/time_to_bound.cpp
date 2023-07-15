// Time to bound safety for velocity based teleoperation
// Author: Rafael Cabral Date: 24.08.2022

#include <time_to_bound/time_to_bound.h>
#include <iostream>

namespace time_to_bound
{

  TimeToBound::TimeToBound(double p_lb, double t2b_min_p, Eigen::Vector3d x_min, Eigen::Vector3d x_max,
                           double q_lb, double t2b_min_q, Eigen::VectorXd q_min, Eigen::VectorXd q_max,
                           double s_lb, double t2b_min_s) : p_lb(p_lb), t2b_min_p(t2b_min_p), x_min(x_min), x_max(x_max),
                                                            q_lb(q_lb), t2b_min_q(t2b_min_q), q_min(q_min), q_max(q_max),
                                                            s_lb(s_lb), t2b_min_s(t2b_min_s)
  {

    std::cout << "Created TimeToBound object" << std::endl;
  };

  Eigen::VectorXd TimeToBound::getSafeJointVelocity(const Eigen::Matrix<double, 6, 1> &twist_d,
                                                    const Eigen::Matrix<double, 4, 4> &O_T_EE,
                                                    const Eigen::VectorXd &q,
                                                    const Eigen::VectorXd &dq,
                                                    const Eigen::MatrixXd &jacobian,
                                                    const Eigen::MatrixXd &jacobianTimeVariation)
  {
    const int n = q.size();
    const int m = jacobian.rows();

    // calculate svd and pseudoinverse
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_full(jacobian, Eigen::ComputeThinU |
                                                             Eigen::ComputeThinV);
    Eigen::VectorXd s(m);
    s = svd_full.singularValues();

    // Moore-Penrose Pinv
    Eigen::MatrixXd jacobian_pinv(m, n);
    jacobian_pinv = svd_full.matrixV() *
                    svd_full.singularValues().cwiseInverse().asDiagonal() *
                    svd_full.matrixU().transpose();

    // joint position delta through diff ik (pinv, identity weight)
    Eigen::VectorXd dq_d(n);
    dq_d = jacobian_pinv * twist_d;

    // derivative of singular values in desired direction of motion
    Eigen::VectorXd s_dot(m);
    s_dot =
        (svd_full.matrixU().transpose() * jacobianTimeVariation * svd_full.matrixV()).diagonal();

    // // tau bounds

    // add position distance (box)
    Eigen::Vector3d x = O_T_EE.block(0, 3, 3, 1);
    Eigen::VectorXd pos_d(6);
    pos_d << x.array() - (x_min.array() + p_lb),
        (x_max.array() - p_lb) - x.array();
    Eigen::VectorXd pos_d_dot(6);
    pos_d_dot << twist_d.head(3), -twist_d.head(3);

    // add singular value distance
    Eigen::VectorXd s_d(s.size());
    s_d << s.array() - s_lb;
    Eigen::VectorXd s_d_dot(s.size());
    s_d_dot << s_dot;

    // add joint limit distance
    Eigen::VectorXd q_d(2 * n);
    q_d << q.array() - (q_min.array() + q_lb), (q_max.array() - q_lb) - q.array();
    Eigen::VectorXd q_d_dot(2 * n);
    q_d_dot << dq_d, -dq_d;

    // combining bounds
    Eigen::VectorXd d(pos_d.size() + s_d.size() + q_d.size());
    d << pos_d, s_d, q_d;

    Eigen::VectorXd d_dot(d.size());
    d_dot << pos_d_dot, s_d_dot, q_d_dot;

    Eigen::VectorXd t2b_min(d.size());

    t2b_min << t2b_min_p * Eigen::VectorXd::Ones(pos_d.size()),
        t2b_min_s * Eigen::VectorXd::Ones(s_d.size()),
        t2b_min_q * Eigen::VectorXd::Ones(q_d.size());

    // calculate safety factor
    double k = 1.;
    for (int i = 0; i < d.size(); i++)
    {
      if (d_dot(i) >= 0)
      {
        continue;
      }
      else if (d(i) < 0)
      {
        k = 0;
        break;
      }
      else
      {
        double k_i = -d(i) / (d_dot(i) * t2b_min(i));
        k = std::min(k_i, k);
      }
    }

    return k * dq_d;
  }

} // namespace time_to_bound
