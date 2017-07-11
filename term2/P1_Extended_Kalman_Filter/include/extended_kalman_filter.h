#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "dynamics.h"
#include "measurements.h"

using namespace Eigen;

class KalmanFilter {
public:

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  KalmanFilter();

  virtual ~KalmanFilter();

  void Init(Eigen::VectorXd&, Eigen::MatrixXd&);
  
  void Predict(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
  
  template<typename M>
  void Update(const Eigen::VectorXd& z, M& measurement);


};

template<typename M>
void KalmanFilter::Update(const VectorXd &z, M& measurement) {
    
    MatrixXd R = measurement.getR();
    MatrixXd H = measurement.getH(x_);
    VectorXd y = measurement.residual(z, x_);
    
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd K = P_ * Ht * S.inverse();
    
    x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H) * P_;
}


#endif

