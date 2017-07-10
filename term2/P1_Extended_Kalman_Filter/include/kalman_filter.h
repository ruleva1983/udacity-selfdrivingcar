#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "motion.h"
#include "lidar.h"
#include "radar.h"

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



};

#endif /* KALMAN_FILTER_H_ */
