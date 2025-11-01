#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    Eigen::Vector3d getState() const { return X_; }
    Eigen::Matrix3d getCovariance() const { return P_; }

    //Kalman'ın 2 temel adımı.
    void predict(double v, double omega, double dt);
    void update(const Eigen::VectorXd& Z);

private:
    //Hesaplamalar için gerekli matrislerimiz ve vektörlerimiz.
    Eigen::Vector3d X_; //State vector: [x, y, theta]
    Eigen::Matrix3d P_; //Covariance matrix
    Eigen::Matrix3d A_; //State transition matrix
    Eigen::Matrix3d Q_; //Process noise covariance
    Eigen::MatrixXd H_; //Measurement matrix
    Eigen::MatrixXd R_; //Measurement noise covariance
};

#endif