#include "extended_kalman_filter.h"
#include "tools.h"
#include <iostream>

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
