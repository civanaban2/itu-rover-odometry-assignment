#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    Eigen::Vector3d getState() const { return X_; }
    Eigen::Matrix3d getCovariance() const { return P_; }
    void predict(double v, double omega, double dt);
    void update(const Eigen::VectorXd& Z);

private:
    Eigen::Vector3d X_; 
    Eigen::Matrix3d P_; 
    Eigen::Matrix3d A_; 
    Eigen::Matrix3d Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

#endif