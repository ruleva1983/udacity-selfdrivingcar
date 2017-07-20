#ifndef FUSION_H_
#define FUSION_H_

#include <iostream>

#include "tools.h"
#include "constants.h"
#include "measurements.h"
#include "Eigen/Dense"

using namespace std;

template <class FilterType, class DynamicsType>
class SensorFusion {
public:

  SensorFusion();

  virtual ~SensorFusion();

  void Initialize(const MeasurementPackage &measurement_pack);
  
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  
  FilterType filter;
  DynamicsType dynamicaModel;
  
  Eigen::VectorXd X_;
  Eigen::MatrixXd P_;
  
  Eigen::VectorXd X_pred; 
  Eigen::MatrixXd P_pred; 
  
  Eigen::VectorXd X_meas;
  Eigen::MatrixXd P_meas;
  
  double epsilon;
  
private:
  bool is_initialized_;
  long long previous_timestamp_;
  
  Lidar lidar_model;
  Radar radar_model;
  
  std::vector<double> NIS_constistence;
  
  
};


template <class FilterType, class DynamicsType>
SensorFusion<FilterType, DynamicsType>::SensorFusion() : is_initialized_(false) {
    NIS_constistence = std::vector<double> (0);
}

template <class FilterType, class DynamicsType>
SensorFusion<FilterType, DynamicsType>::~SensorFusion() {}


template <class FilterType, class DynamicsType>
void SensorFusion<FilterType, DynamicsType>::Initialize(const MeasurementPackage &measurement_pack)
{
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        radar_model.initialize(measurement_pack.raw_measurements_, X_, P_);
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        lidar_model.initialize(measurement_pack.raw_measurements_, X_, P_);
}


template <class FilterType, class DynamicsType>
void SensorFusion<FilterType, DynamicsType>::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    cout << "State initialization based on the first measurement..." << endl;  
    Initialize(measurement_pack);
    return;
  }
  
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / time_normalizer;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  cout << "Predict phase " << dt << " seconds" << endl;
  
  
  X_pred = X_;
  P_pred = P_;
  filter.predict(X_pred, P_pred, dynamicaModel, dt);
  
  
  cout << "Update phase..."  << endl << endl;
  
  X_meas = X_pred;
  P_meas = P_pred;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
      filter.template update<Radar>(radar_model, X_meas, P_meas);
  else
      filter.template update<Lidar>(lidar_model, X_meas , P_meas);
  
  
  
  Eigen::VectorXd z_meas = measurement_pack.raw_measurements_;
  filter.apply_kalman(z_meas, X_pred, P_pred, X_meas, P_meas, X_, P_);
  
  epsilon = Tools::NIS(z_meas, X_meas, P_meas);
  NIS_constistence.push_back(epsilon);

}
#endif
