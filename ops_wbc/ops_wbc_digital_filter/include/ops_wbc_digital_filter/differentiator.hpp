#ifndef __OPS_WBC_DIGITAL_FILTER_DIFFERENTIATOR_HPP
#define __OPS_WBC_DIGITAL_FILTER_DIFFERENTIATOR_HPP
#include <memory>
#include <Eigen/Dense>

namespace ops_wbc_digital_filter
{
    class Differentiator 
    {
        public:
            explicit Differentiator() {}
            virtual ~Differentiator() = default;
            virtual void init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) = 0;
            virtual void apply(const Eigen::VectorXd& q) = 0;
            virtual void copyDerivativeValueTo(Eigen::VectorXd& dq) = 0;
    };

    using DifferentiatorPtr = std::shared_ptr<Differentiator>;
}
#endif // __OPS_WBC_DIGITAL_FILTER_DIFFERENTIATOR_HPP
