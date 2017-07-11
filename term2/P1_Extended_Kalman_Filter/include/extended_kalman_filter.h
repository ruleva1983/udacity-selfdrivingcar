#ifndef EXTENDED_KALMAN_FILTER_H_
#define EXTENDED_KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "dynamics.h"
#include "measurements.h"


class KalmanFilter {
public:

  // state
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  KalmanFilter();

  virtual ~KalmanFilter();

  void Init(Eigen::VectorXd&, Eigen::MatrixXd&);
  
  void Predict(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
  
  void Update(const Eigen::VectorXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);
  
  template<typename M>
  void Update2(const Eigen::VectorXd& z, M& measurement);

  void UpdateLidar(const Eigen::VectorXd&, Lidar&);
  void UpdateRadar(const Eigen::VectorXd& , Radar&);


};

#endif /* KALMAN_FILTER_H_ */

