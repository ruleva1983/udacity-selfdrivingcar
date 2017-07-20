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
    void transform(const T&, double);
    
    void evalState(Eigen::VectorXd&, Eigen::MatrixXd&);
    
    Eigen::MatrixXd Points;
    Eigen::MatrixXd Points_pred;
    Eigen::MatrixXd Points_meas;
    Eigen::VectorXd weights;
    
private:
    int nb_points;
};



template <typename T>
void PointGenerator::transform(const T& Model, double dt){
    Eigen::MatrixXd Transformed_Points(Model.nb_state_variables, nb_points);
    for (int i = 0 ; i < nb_points ; ++i){
        Eigen::VectorXd Point = Points.col(i);
        Transformed_Points.col(i) = Model.transform(Point, dt);
    }
    Points_pred = Transformed_Points;
}


#endif
