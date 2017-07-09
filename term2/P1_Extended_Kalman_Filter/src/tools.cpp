#include <iostream>
#include "tools.h"
#include "measurement_package.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd RMSE(4);
    return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}



void Tools::EncodeLine(MeasurementPackage& meas_package, vector<VectorXd>& ground_truth, string& sensor_measurement){
    
    istringstream iss(sensor_measurement);
    long long timestamp;
    string sensor_type;
    	  
    iss >> sensor_type;
          
    if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        
        float x, y;
        iss >> x >> y >> timestamp;
        
        meas_package.raw_measurements_ << x, y;
        meas_package.timestamp_ = timestamp;
    } 
    else if (sensor_type.compare("R") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        
        float ro, theta, ro_dot;
        iss >> ro >> theta >> ro_dot >> timestamp;
        
        meas_package.raw_measurements_ << ro,theta, ro_dot;
        meas_package.timestamp_ = timestamp;
    }
    
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);
}











