#ifndef __OPS_WBC_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
#define __OPS_WBC_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP

#include <memory>
#include <Eigen/Dense>

#include "kalman_filter/normal_distribution.hpp"

namespace ops_wbc_kalman_filter
{
    class ExtendedKalmanFilter
    {
        public:
            void setRandomVariables(const NormalDistributionPtr& state,
                            const NormalDistributionPtr& uncertainty,
                            const NormalDistributionPtr& msr_noise);

        private:
            NormalDistributionPtr state_;
            NormalDistributionPtr predicted_state_;
            NormalDistributionPtr uncertainty_;
            NormalDistributionPtr msr_noise_;
    };
    using ExtendedKalmanFilterPtr = std::shared_ptr<ExtendedKalmanFilter>;
}
#endif // __OPS_WBC_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
