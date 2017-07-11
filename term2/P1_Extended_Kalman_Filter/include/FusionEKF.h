#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "measurements.h"
#include "dynamics.h"
#include "extended_kalman_filter.h"


class SensorFusion {
public:

  SensorFusion();

  virtual ~SensorFusion();

  void Initialize(const MeasurementPackage &measurement_pack);
  
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  
  KalmanFilter EKF;
private:
  bool is_initialized_;
  long long previous_timestamp_;
  DynamicalModel dynamics;
  Lidar lidar_model;
  Radar radar_model;
  
};

#endif /* FusionEKF_H_ */
