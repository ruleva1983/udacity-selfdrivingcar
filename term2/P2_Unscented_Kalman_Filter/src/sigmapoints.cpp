#include "sigmapoints.h"
#include "constants.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cassert>

using namespace std;


PointGenerator::PointGenerator(){}
PointGenerator::~PointGenerator(){}

void PointGenerator::generate(Eigen::VectorXd& Mean, Eigen::MatrixXd& Covariance)
{
    int nb_dof = Mean.size();
    Points = Eigen::MatrixXd(nb_dof,2*nb_dof + 1);
    Points.col(0)  = Mean;
    nb_points = 1;

    Eigen::MatrixXd A = Covariance.llt().matrixL();

    
    for (int i=0 ; i<nb_dof ; ++i){
        Points.col(i+1) = Mean + std::sqrt(lambda + nb_dof)*A.col(i);
        Points.col(i+1+nb_dof) = Mean - std::sqrt(lambda + nb_dof)*A.col(i);
        nb_points += 2;
    }
}


void PointGenerator::evalState(Eigen::VectorXd& X_, Eigen::MatrixXd& P_){
    
    int nb_dof = Points_pred.rows();
    int nb_dof_noise = Points.rows();
    
    Eigen::VectorXd X_acc = Eigen::VectorXd::Zero(nb_dof);
    Eigen::MatrixXd P_acc = Eigen::MatrixXd::Zero(nb_dof, nb_dof);

    weights = Eigen::VectorXd::Zero (Points_pred.cols());
    weights(0) = lambda/(lambda + nb_dof_noise);
    for (int i = 1 ; i < Points_pred.cols() ; ++i){
        weights(i) = 0.5/(lambda + nb_dof_noise);
    }
    
    for (int i = 0 ; i < Points_pred.cols() ; ++i)
        X_acc += weights(i)*Points_pred.col(i);
    
    for (int i = 0 ; i < Points_pred.cols() ; ++i){
        Eigen::VectorXd x_diff = Points_pred.col(i) - X_acc;
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        P_acc += weights(i) * x_diff*x_diff.transpose();
    }
    
    X_ = X_acc;
    P_ = P_acc;
}
