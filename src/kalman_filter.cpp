#include "iturover_odometry_assignment/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {
    X_.setZero();
    P_ = Eigen::Matrix3d::Identity() * 0.1;
    Q_ = Eigen::Matrix3d::Identity();
    Q_(0, 0) = 0.001;
    Q_(1, 1) = 0.001;
    Q_(2, 2) = 0.005;
    H_.resize(2, 3);
    H_.setZero();
    H_(0, 2) = 1.0;
    R_.resize(2, 2);
    R_.setZero();
    R_(0, 0) = 0.0001;
    R_(1, 1) = 0.0001;
}

void KalmanFilter::predict(double v, double omega, double dt) {
    double theta = X_(2);
    double dx, dy;

    if (std::abs(omega) < 1e-6)
	{
        dx = v * dt * std::cos(theta);
        dy = v * dt * std::sin(theta);
    }
	else
	{
        double r = v / omega;
        dx = r * (std::sin(theta + omega * dt) - std::sin(theta));
        dy = r * (-std::cos(theta + omega * dt) + std::cos(theta));
    }
    X_(0) += dx;
    X_(1) += dy;
    X_(2) += omega * dt;
    A_.setIdentity();
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& Z) {
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    Eigen::VectorXd Y_pred(2);
    Y_pred(0) = X_(2);
    Y_pred(1) = 0.0;
    Eigen::VectorXd y = Z - Y_pred;
    if (y(0) > M_PI)
	{
        y(0) -= 2 * M_PI;
    }
	else if (y(0) < -M_PI)
	{
        y(0) += 2 * M_PI;
    }
    X_ = X_ + K * y;
    P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
}
