/*************************************************************************
	> File Name: kalman_filter.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 13 Dec 2017 01:20:48 PM PST
 ************************************************************************/

#include <iostream>
#include "wam_dmp_controller/kalman_filter.hpp"
#include "wam_dmp_controller/exception.hpp"

using namespace wam_dmp_controller;
using namespace std;

void SISE_KalmanFilter::setRandomVariables(const NormalDistributionPtr& state,
                                      const NormalDistributionPtr& state_uncertainty,
                                      const NormalDistributionPtr& msr_noise)
{
    state_ = state;
    state_uncertainty_ = state_uncertainty;
    msr_noise_ = msr_noise;
    
    predicted_state_.reset(new NormalDistribution(state_->getMean(), state_->getVariance()));
}

void SISE_KalmanFilter::InitializeLinearModel(const Eigen::MatrixXd& coeff_of_mean,
                                  const Eigen::MatrixXd& coeff_of_ctrl_data,
                                  const Eigen::MatrixXd& coeff_of_msr_data)
{
    if (coeff_of_mean.rows() != coeff_of_mean.cols())
    {
        std::stringstream msg;
        msg << "coeff_of_mean.rows() != coeff_of_mean.cols()" << std::endl
            << "coeff_of_mean.rows() : " << coeff_of_mean.rows() << std::endl
            << "coeff_of_mean.cols() : " << coeff_of_mean.cols();
        throw wam_dmp_controller::Exception("KalmanFilter::setLinearModel", msg.str());
    }

    if (coeff_of_msr_data.rows() != coeff_of_msr_data.cols())
    {
        std::stringstream msg;
        msg << "coeff_of_mean.rows() != coeff_of_mean.cols()" << std::endl
            << "coeff_of_mean.rows() : " << coeff_of_mean.rows() << std::endl
            << "coeff_of_mean.cols() : " << coeff_of_mean.cols();
        throw wam_dmp_controller::Exception("KalmanFilter::setLinearModel", msg.str());
    }
    

    A_ = coeff_of_mean;
    B_ = coeff_of_ctrl_data;
    C_ = coeff_of_msr_data;
    state_dim_ = coeff_of_mean.rows();
    measurement_dim_ = coeff_of_msr_data.rows();
    input_dim_ = coeff_of_ctrl_data.cols();
}

void SISE_KalmanFilter::updateCtrlMatrix(const Eigen::MatrixXd& coeff_of_ctrl_data)
{
    B_ = coeff_of_ctrl_data; 
}

void SISE_KalmanFilter::estimate(const Eigen::MatrixXd& ctrl_input, const Eigen::MatrixXd& msr_data_curr, const Eigen::MatrixXd& msr_data_last, NormalDistributionPtr& state)
{
    /*
    Eigen::MatrixXd mean = A_ * state_->getMean() + B_ * ctrl_input;
    Eigen::MatrixXd variance = A_ * state_->getVariance() * A_.transpose() + state_uncertainty_->getVariance();
    predicted_state_->set(mean, variance);

    Eigen::MatrixXd kalman_gain = variance * C_.transpose() * (C_ * variance * C_.transpose() + msr_noise_->getVariance()).inverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(variance.rows(), variance.cols());
    mean = mean + kalman_gain * (msr_data - C_ * mean);
    variance = (I - kalman_gain * C_) * variance;

    state->set(mean, variance);
    state_ = state;
    */
    Q_ = C_ * P_  * C_.transpose(  ) + C_ * state_uncertainty_->getVariance() * C_.transpose() + msr_noise_->getVariance();
    Q_inverse_ = Q_.inverse();
    H_ = B_.transpose() * C_.transpose() * Q_inverse_ * C_ * B_;
    H_ = H_.inverse() * B_.transpose() * C_.transpose() * Q_inverse_;
    u_mean_ = H_ * (msr_data_curr - C_ * A_ * state_->getMean());

    P_uu_ = H_ * Q_ * H_.transpose();
    P_ux_ = -H_ * C_ * A_ * P_xx_;
    P_.block(0, 0, state_dim_, state_dim_) = P_xx_;
    P_.block(state_dim_ + 1, 0, input_dim_, state_dim_) = P_ux_;
    P_.block(0, state_dim_ + 1, state_dim_, input_dim_) = P_ux_.transpose();
    P_.block(state_dim_ + 1, state_dim_ + 1, input_dim_, input_dim_) = P_uu_;
    G_.block(0, 0, state_dim_, state_dim_) = A_;
    G_.block(0, state_dim_ + 1, state_dim_, input_dim_) = B_;
    K_.block(0, 0, measurement_dim_, state_dim_) = C_;
    K_.block(0, measurement_dim_ + 1,  measurement_dim_, input_dim_) = Eigen::MatrixXd::Zero(measurement_dim_, input_dim_);
    S_ = G_ * P_ * G_.transpose();
    T_ = G_ * P_ * K_.transpose();
    U_ = K_ * P_ * K_.transpose();
    L_ = T_ * U_.inverse();

    x_mean_ = A_ * state_->getMean() + B_ * u_mean_ + L_ * (msr_data_last - C_ * state_->getMean());
    P_xx_ = S_ - L_ * U_.inverse() * input_->getVariance();
    
    state_->set(x_mean_, P_xx_);
    input_->set(u_mean_, P_uu_);
}

void SISE_KalmanFilter::checkMatrixSize(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data)
{
    std::string src = "KalmanFilter::checkMatrixSize";

    if (A_.cols() != state_->getMean().rows())
    {
        std::stringstream msg;
        msg << "A_.cols() != state_->getMean().rows()" << std::endl
            << "A_.cols() : " << A_.cols() << std::endl
            << "state_->getMean().rows() : " << state_->getMean().rows();
        throw wam_dmp_controller::Exception(src, msg.str());
    }

    if(B_.cols() != ctrl_data.rows())
    {
        std::stringstream msg;

        msg << "B_.cols() != ctrl_data.rows()" << std::endl
            << "B_.cols() : " << B_.cols() << std::endl
            << "ctrl_data.rows() : " << ctrl_data.rows();

        throw wam_dmp_controller::Exception(src, msg.str());
    }

    if(A_.cols() != state_->getVariance().rows())
    {
        std::stringstream msg;

        msg << "A_.cols() != state_->getVariance().rows()" << std::endl
            << "A_.cols() : " << A_.cols() << std::endl
            << "state_->getVariance().rows() : " << state_->getVariance().rows();

        throw wam_dmp_controller::Exception(src, msg.str());
    }

    if(A_.rows() != state_->getVariance().cols())
    {
        std::stringstream msg;

        msg << "A_.rows() != state_->getVariance().cols()" << std::endl
            << "A_.rows() : " << A_.rows() << std::endl
            << "state_->getVariance().cols() : " << state_->getVariance().cols();

        throw wam_dmp_controller::Exception(src, msg.str());
    }
}
