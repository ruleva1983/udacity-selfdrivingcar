#include "sigmapoints.h"
#include <algorithm>
#include <cmath>
#include <iostream>
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
    
    Eigen::VectorXd X_acc = Eigen::VectorXd::Zero(nb_dof);
    Eigen::MatrixXd P_acc = Eigen::MatrixXd::Zero(nb_dof, nb_dof);

    double w0 = lambda/(lambda + nb_dof);
    double wi = (1.0/(2.0*(lambda + nb_dof)));
    
    X_acc += w0*Points_pred.col(0);
    
    for (int i = 1 ; i < Points_pred.cols() ; ++i)
        X_acc += wi*Points_pred.col(i);
    
    P_acc += w0 * (Points_pred.col(0) - X_acc)*(Points_pred.col(0) - X_acc).transpose();
    
    for (int i = 1 ; i < Points_pred.cols() ; ++i)
        P_acc += wi * (Points_pred.col(i) - X_acc)*(Points_pred.col(i) - X_acc).transpose();

    X_ = X_acc.head(nb_dof);
    P_ = P_acc.block(0,0,nb_dof,nb_dof);
    
}
