#include "measurements.h"
#include <iostream>

using namespace std;

//Abstract Function Functions

measurement::measurement(){}
measurement::~measurement(){}


//Radar Functions


Radar::Radar(){}
Radar::~Radar(){}

Eigen::MatrixXd Radar::getR() const{
    Eigen::MatrixXd R(3,3);
    R << noise_radar_rho, 0.0,               0.0,
         0.0,             noise_radar_theta, 0.0,
         0.0,              0.0,               noise_radar_rho;
    return R;
}


Eigen::MatrixXd Radar::getH(const Eigen::VectorXd& state) const{
    
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


Eigen::VectorXd Radar::residual(const Eigen::VectorXd& z, Eigen::VectorXd& x_) const
{
    Eigen::VectorXd z_hat = Eigen::VectorXd::Zero(3);

    const double x = x_(0);
    const double y = x_(1);
    const double vx = x_(2);
    const double vy = x_(3);
    const double c1 = std::sqrt(x*x + y*y);

    if (c1 > 0.001)
        z_hat << c1, std::atan2(y, x), (x * vx + y * vy) / c1;
    
    cout << "z_hat " <<  z_hat << endl;
    Eigen::VectorXd res = z - z_hat;

    double x_mod = std::fmod(res(1) + M_PI, 2*M_PI);
    if (x_mod < 0.0)
        x_mod += 2*M_PI;
    res(1) = x_mod - M_PI;
    
    return res;
}


//Lidar Functions

Lidar::Lidar() {}
Lidar::~Lidar() {}

Eigen::MatrixXd Lidar::getR() const{
    Eigen::MatrixXd R(2,2);
    R << noise_lidar, 0.0,
         0.0,         noise_lidar;
    return R;
}

Eigen::MatrixXd Lidar::getH(const Eigen::VectorXd& state) const{
    Eigen::MatrixXd H(2,4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;
    return H;
}

Eigen::VectorXd Lidar::residual(const Eigen::VectorXd& z, Eigen::VectorXd& x_) const
{
    Eigen::VectorXd res = z - Lidar::getH(x_)*x_;
    return res;
}

