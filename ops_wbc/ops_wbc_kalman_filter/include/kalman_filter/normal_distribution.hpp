#ifndef __OPS_WBC_KALMAN_FILTER_NORMAL_DISTRIBUTION_HPP
#define __OPS_WBC_KALMAN_FILTER_NORMAL_DISTRIBUTION_HPP

#include <memory>
#include <Eigen/Dense>

namespace ops_wbc_kalman_filter
{
    class NormalDistribution
    {
        public:
            NormalDistribution() {}
            NormalDistribution(const Eigen::MatrixXd& mean, const Eigen::MatrixXd& variance) : mean_(mean), variance_(variance) {}
            void set(const Eigen::MatrixXd& mean, const Eigen::MatrixXd& variance)
            {
                mean_ = mean;
                variance_ = variance;
            }

            const Eigen::MatrixXd& getMean()
            {
                return mean_;
            }

            const Eigen::MatrixXd& getVariance()
            {
                return variance_;
            }

        private:
            Eigen::MatrixXd mean_;
            Eigen::MatrixXd variance_;
    };
    using NormalDistributionPtr = std::shared_ptr<NormalDistribution>;
}

#endif // __OPS_WBC_KALMAN_FILTER_NORMAL_DISTRIBUTION_HPP
