#include "measurements.h"
#include "Eigen/Dense"

#include <iostream>
#include <cmath>

using namespace std;


//Abstract Function Functions

measurement::measurement(){}
measurement::~measurement(){}
void measurement::initialize(const Eigen::VectorXd&, Eigen::VectorXd&, Eigen::MatrixXd&) const {};



void measurement::evalState(Eigen::MatrixXd& SigmaPoints, Eigen::VectorXd& X_, Eigen::MatrixXd& P_) const
{
    int nb_dof = SigmaPoints.rows();
    
    Eigen::VectorXd X_acc = Eigen::VectorXd::Zero(nb_dof);
    Eigen::MatrixXd P_acc = Eigen::MatrixXd::Zero(nb_dof, nb_dof);

    double lambda = -4.0;
    double w0 = lambda/(lambda + 7);
    double wi = 0.5/(lambda + 7);
    
    X_acc += w0*SigmaPoints.col(0);
    
    for (int i = 1 ; i < SigmaPoints.cols() ; ++i)
        X_acc += wi*SigmaPoints.col(i);
    
    Eigen::VectorXd z_diff = SigmaPoints.col(0) - X_acc;
    if (nb_dof==3){
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    }
    P_acc += w0 * z_diff*z_diff.transpose();
    for (int i = 1 ; i < SigmaPoints.cols() ; ++i){
        z_diff = SigmaPoints.col(i) - X_acc;
        if (nb_dof==3){
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        }
        P_acc += wi * z_diff*z_diff.transpose();
    }

    X_ = X_acc;
    P_ = P_acc;
    
}


//Lidar

Lidar::Lidar() {
    X_initialize = Eigen::VectorXd::Zero(5);
    P_initialize = Eigen::MatrixXd::Zero(5,5);
    for (int i = 0 ; i < 5 ; ++i)
        P_initialize(i,i) = 1.0;
}
Lidar::~Lidar() {}

void Lidar::initialize(const Eigen::VectorXd& measurements, Eigen::VectorXd& X_, Eigen::MatrixXd& P_) const
{
    double x = measurements[0];
    double y = measurements[1];
    X_ = X_initialize;
    X_[0] = x;
    X_[1] = y;
    P_ = P_initialize;
}


Eigen::MatrixXd Lidar::transform(Eigen::MatrixXd& Points) const
{
    cout << "Lidar measurement..." << endl;
    int nb_points = Points.cols();
    Eigen::MatrixXd TrasformedPoints = Eigen::MatrixXd::Zero(nb_meas, nb_points);
    
    for (int i = 0 ; i < nb_points ; ++i){
        Eigen::VectorXd Point = Points.col(i);
        TrasformedPoints.col(i) << Point(0), Point(1);
    }
    return TrasformedPoints;
}

void Lidar::AddNoise(Eigen::MatrixXd& P_) const{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2);
    R << noise_lidar, 0, 0, noise_lidar;
    P_ += R;
}



//Radar

Radar::Radar() {
    X_initialize = Eigen::VectorXd::Zero(5);
    P_initialize = Eigen::MatrixXd::Zero(5,5);
    
    //TODO Different initialization for Radar than for lidar
    for (int i = 0 ; i < 5 ; ++i)
        P_initialize(i,i) = 1.0;
}
Radar::~Radar() {}


void Radar::initialize(const Eigen::VectorXd& measurements, Eigen::VectorXd& X_, Eigen::MatrixXd& P_) const
{
    double rho = measurements[0];
    double theta = measurements[1];
    double rhodot = measurements[2];
    X_ = X_initialize;
    X_[0] = rho*std::cos(theta);
    X_[1] = rho*std::sin(theta);;
    P_ = P_initialize;
}



Eigen::MatrixXd Radar::transform(Eigen::MatrixXd& Points) const
{
    
    int nb_points = Points.cols();
    Eigen::MatrixXd TrasformedPoints = Eigen::MatrixXd::Zero(nb_meas, nb_points);
    
    for (int i = 0 ; i < nb_points ; ++i )
    {
        Eigen::VectorXd Point = Points.col(i);
        double x = Point(0), y = Point(1), v = Point(2), psi = Point(3); 
        double rho = std::sqrt(x*x + y*y);
        double theta = std::atan2(y, x);
        double rhodot = v*(x*std::cos(psi) + y*std::sin(psi))/rho;
        TrasformedPoints.col(i) << rho, theta, rhodot;
    }
    return TrasformedPoints;
}

void Radar::AddNoise(Eigen::MatrixXd& P_) const{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
    R << noise_radar_rho, 0, 0,
         0, noise_radar_theta, 0,
         0, 0, noise_radar_rho;
    P_ += R;
}



