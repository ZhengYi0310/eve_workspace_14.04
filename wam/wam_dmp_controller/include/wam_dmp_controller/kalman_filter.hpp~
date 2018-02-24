#ifndef __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP
#define __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP

#include <memory>
#include <Eigen/Dense>

#include "kalman_filter/normal_distribution.hpp"

namespace ops_wbc_kalman_filter
{
    class KalmanFilter
    {
        public:
            void setRandomVariables(const NormalDistributionPtr& state,
                                    const NormalDistributionPtr& uncertainty,
                                    const NormalDistributionPtr& msr_noise);
            void setLinearModel(const Eigen::MatrixXd& coeff_of_mean,
                                const Eigen::MatrixXd& coeff_of_ctrl_data,
                                const Eigen::MatrixXd& coeff_of_msr_data);
            void estimate(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data, NormalDistributionPtr& state);

            const NormalDistributionPtr& getState() const 
            {
                return state_;
            }

            const NormalDistributionPtr& getPredictedState() const 
            {
                return predicted_state_;
            }
        
        private:
            void checkMatrixSize(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data);
            
            NormalDistributionPtr state_;
            NormalDistributionPtr predicted_state_;
            NormalDistributionPtr uncertainty_;
            NormalDistributionPtr msr_noise_;

            Eigen::MatrixXd A_;
            Eigen::MatrixXd B_;
            Eigen::MatrixXd C_;
    };
    using KalmanFilterPtr = std::shared_ptr<KalmanFilter>;
}
#endif // __OPS_WBC_KALMAN_FILTER_KALMAN_FILTER_HPP
