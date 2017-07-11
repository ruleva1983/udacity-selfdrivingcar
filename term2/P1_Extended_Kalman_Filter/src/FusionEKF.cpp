#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

SensorFusion::SensorFusion() : is_initialized_(false) {}

SensorFusion::~SensorFusion() {}

void SensorFusion::Initialize(const MeasurementPackage &measurement_pack)
{
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "Initialization with first measurement: " << measurement_pack.sensor_type_ << endl;
    double x, y;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        double rho = measurement_pack.raw_measurements_[0];
        double phi = measurement_pack.raw_measurements_[1];
        x = rho * cos(phi);
        y = rho * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        x = measurement_pack.raw_measurements_[0];
        y = measurement_pack.raw_measurements_[1];
    }
    
    VectorXd x_(4);
    x_ << x, y, 0, 0;
    
    MatrixXd P_(4,4);
    P_ << 1.0,  0.0,   0.0,  0.0,
          0.0,  1.0,   0.0,  0.0,
          0.0,  0.0,   1.0,  0.0,
          0.0,  0.0,   0.0,  1.0; 
          
    EKF.Init(x_, P_);
}


void SensorFusion::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    Initialize(measurement_pack);
    return;
  }
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  EKF.Predict(dynamics.getF(dt), dynamics.getQ(dt));

  cout << "New measurement: " << measurement_pack.sensor_type_ << endl;
  
  VectorXd z = measurement_pack.raw_measurements_;

    //TODO Debug from here
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      EKF.UpdateRadar(z, radar_model);
  } else {
      EKF.UpdateLidar(z, lidar_model);
  }

}
