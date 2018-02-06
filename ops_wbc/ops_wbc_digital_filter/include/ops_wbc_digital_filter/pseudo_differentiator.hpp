#ifndef __OPS_WBC_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP
#define __OPS_WBC_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP

#include <Eigen/Dense>
#include "ops_wbc_digital_filter/differentiator.hpp"

namespace ops_wbc_digital_filter
{
    class PseudoDifferentiator : public Differentiator 
    {
        public:
            explicit PseudoDifferentiator(double period, double cutoff_freq);

            virtual void init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) override;
            virtual void apply(const Eigen::VectorXd& q) override;
            virtual void copyDerivativeValueTo(Eigen::VectorXd& dq) override
            {
                dq = dq_;
            }

        private:
            Eigen::VectorXd q_;
            Eigen::VectorXd dq_;
            Eigen::VectorXd pre_q_;
            Eigen::VectorXd pre_dq_;
            double period_;
            double T_;

            double coeff1_;
            double coeff2_;
    };
}
#endif // __OPS_WBC_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP
