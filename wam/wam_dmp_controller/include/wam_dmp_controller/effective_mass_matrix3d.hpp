#ifndef __WAM_DMP_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP
#define __WAM_DMP_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP

#include <Eigen/Dense>

namespace wam_dmp_controller
{
  inline void compute(const Eigen::Matrix3d& lambda_inv, Eigen::Matrix3d& lambda, double thresh)
  {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(lambda_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d S = svd.singularValues().asDiagonal();
    for(uint32_t i = 0; i < 3; ++i)
    {
      if(S.coeff(i, i) < thresh)
      {
        S.coeffRef(i, i) = thresh;
      }
    }

    lambda = svd.matrixV() * S.inverse() * svd.matrixU().transpose();
  }
} // namespace ahl_ctrl

#endif // __AHL_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP

