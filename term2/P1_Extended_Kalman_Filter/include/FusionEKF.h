#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
#include "motion.h"
#include "lidar.h"
#include "radar.h"

class FusionEKF {
public:

  FusionEKF();

  virtual ~FusionEKF();

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  KalmanFilter ekf_;

private:
    
  void Initialize(const MeasurementPackage &measurement_pack);    

  bool is_initialized_;
  long long previous_timestamp_;
  Tools tools;
  //Lidar lidar;
  //Radar radar;
  Motion motion;

};

#endif /* FusionEKF_H_ */
