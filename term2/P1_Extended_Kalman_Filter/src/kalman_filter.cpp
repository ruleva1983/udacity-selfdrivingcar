#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_0, MatrixXd &P_0) {
  x_ = x_0;
  P_ = P_0;
}

void KalmanFilter::Predict(const MatrixXd& F_, const MatrixXd& Q_) {
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd& H_, const Eigen::MatrixXd& R_) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    //Angle normalization idiot only for radar data
    if (z.size() ==3){
    double x_mod = std::fmod(y(1) + M_PI, 2*M_PI);
    if (x_mod < 0.0)
        x_mod += 2*M_PI;
    y(1) = x_mod - M_PI;
    }
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();
    
    x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

