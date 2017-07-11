#include "sigmapoints.h"

#include <algorithm>
#include <cmath>

PointGenerator::PointGenerator(){
    Points = std::vector<Eigen::VectorXd> (0);
    nb_points = 0;
}
PointGenerator::~PointGenerator(){}

void PointGenerator::generate(Eigen::VectorXd& Mean, Eigen::MatrixXd& Covariance)
{
    //TODO check this does not give problems
    // Adding the mean 
    Eigen::VectorXd Copy_Mean = Mean;
    Points.push_back(Copy_Mean);
    nb_points += 1;
    
    Eigen::MatrixXd A = Covariance.llt().matrixL();
    for (int i = 0 ; i < A.cols() ; ++i){
        Points.push_back(Mean - std::sqrt(3)*A.col(i));
        Points.push_back(Mean + std::sqrt(3)*A.col(i));
        nb_points += 2;
    }
}

