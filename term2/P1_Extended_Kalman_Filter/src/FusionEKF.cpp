#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include "constants.h"
#include "lidar.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
}

FusionEKF::~FusionEKF() {}


void FusionEKF::Initialize(const MeasurementPackage &measurement_pack)
{
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "First measurement: " << measurement_pack.sensor_type_ << endl;
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
    P_ << SigmaX, 0.0,   0.0,  0.0,
          0.0,  SigmaX,  0.0,  0.0,
          0.0,  0.0,   SigmaVX, 0.0,
          0.0,  0.0,   0.0,  SigmaVX; 
          
    ekf_.Init(x_, P_);
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    Initialize(measurement_pack);
    return;
  }
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.Predict(motion.getF(dt), motion.getQ(dt));

  cout << "New measurement: " << measurement_pack.sensor_type_ << endl;
  
  VectorXd z = measurement_pack.raw_measurements_;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      Radar radar;
      ekf_.Update(z, radar.getH(ekf_.x_), radar.getR());
  } else {
    Lidar lidar;
    ekf_.Update(z, lidar.getH(), lidar.getR());
  }
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
