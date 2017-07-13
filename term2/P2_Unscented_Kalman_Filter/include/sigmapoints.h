#ifndef SIGMAPOINTS_H_
#define SIGMAPOINTS_H_

#include <vector>
#include "Eigen/Dense"

#include <iostream>

using namespace std;

class PointGenerator{
public:
    PointGenerator();
    ~PointGenerator();
    
    void generate(Eigen::VectorXd&, Eigen::MatrixXd&);
    
    template <typename T>
    void transform(T&, double);
    
    void evalState(Eigen::VectorXd&, Eigen::MatrixXd&);
    
    Eigen::MatrixXd Points;
    Eigen::MatrixXd Points_pred;
    Eigen::MatrixXd Points_meas;
    
private:
    int nb_points;
    double lambda = -4.0;
};



template <typename T>
void PointGenerator::transform(T& Model, double dt){
    Eigen::MatrixXd Transformed_Points(5, nb_points);
    for (int i = 0 ; i < nb_points ; ++i){
        Eigen::VectorXd Point = Points.col(i);
        Transformed_Points.col(i) = Model.transform(Point, dt);
    }
    Points_pred = Transformed_Points;
}


#endif
