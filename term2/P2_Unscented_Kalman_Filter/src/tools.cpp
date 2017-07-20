
#include <iostream>
#include "tools.h"
#include "measurements.h"
#include <cassert>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth
                             ) {
    VectorXd rmse(4);
	rmse << 0,0,0,0;

	assert (estimations.size() == ground_truth.size());
    size_t size = estimations.size();
	for(auto i=0; i < size; ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();
    
	return rmse;
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


void Tools::write_output(ofstream& file, const VectorXd& measurements, const VectorXd& ground_truth, const VectorXd& estimation, const VectorXd& RMSE, double epsilon){
    
    for(int i=0; i<ground_truth.size(); ++i)
        file << ground_truth[i] << ",";
    
    for(int i=0; i<estimation.size(); ++i)
        file << estimation[i] << ",";
    
    for(int i=0; i<2; ++i)
        file << measurements[i] << ",";
    
    for(int i=0; i<RMSE.size(); ++i)
        file << RMSE[i] << ",";
    file << epsilon << std::endl;
}


double Tools::NIS(Eigen::VectorXd& z_meas, Eigen::VectorXd& X_meas , Eigen::MatrixXd& P_meas){
    Eigen::VectorXd residual = z_meas - X_meas;
    double epsilon = residual.transpose()*P_meas.inverse()*residual;
    
    // We normalize in respect to the degrees of freedom
    if (z_meas.size() == 2)
        epsilon /= 5.991;
    else if (z_meas.size() == 3)
        epsilon /= 7.815;

    return epsilon;
}






