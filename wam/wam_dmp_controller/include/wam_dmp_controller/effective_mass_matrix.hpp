#ifndef __WAM_DMP_CONTROLLER_EFFECTIVE_MASS_MATRIX_HPP
#define __WAM_DMP_CONTROLLER_EFFECTIVE_MASS_MATRIX_HPP

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <stdint.h>

namespace wam_dmp_controller
{
  inline void ComputeMassMatrix(const Eigen::MatrixXd& lambda_inv, Eigen::MatrixXd& lambda, double thresh = 0.001, bool damped=true)
  {
    double lambda_ = damped ? 0.2:0.0;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(lambda_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S = lambda_inv;
    S.setZero();
    for(int i = 0; i < sing_vals_.size(); ++i)
    {
      if(sing_vals_(i) < thresh)
      {
        S.coeffRef(i, i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);
      }
    }

    lambda = svd.matrixV() * S * svd.matrixU().transpose();
  }
} // namespace ahl_ctrl

#endif // __AHL_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP

