#include "radar.h"
#include "constants.h"
#include <cmath>
#include <iostream>

Radar::Radar(){}
Radar::~Radar(){}



Eigen::MatrixXd Radar::getR(){
    Eigen::MatrixXd R(3,3);
    R << noise_radar_rho, 0.0,               0.0,
         0.0,             noise_radar_theta, 0.0,
         0.0,              0.0,               noise_radar_rho;
    return R;
}

Eigen::MatrixXd Radar::getH(const Eigen::VectorXd& state){
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,4);
    
    double x = state(0);
    double y = state(1);
    double vx = state(2);
    double vy = state(3);

    double c1 = x*x+y*y;
	double c2 = sqrt(c1);
	double c3 = (c1*c2);
    
    if (c1 > 0.000001){
        H << x/c2,              y/c2,               0,    0,
            -y/c1,              x/c1,               0,    0,
            y*(vx*y - vy*x)/c3, x*(x*vy - y*vx)/c3, x/c2, y/c2;
    }
    return H;
}
