#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <vector>
#include "Eigen/Dense"

class PointGenerator{
public:
    PointGenerator();
    ~PointGenerator();
    
    void generate(Eigen::VectorXd&, Eigen::MatrixXd&);
    
    
    
    
    template <typename T>
    std::vector<Eigen::VectorXd> transform(T&) const;
    
private:
    std::vector<Eigen::VectorXd> Points;
    int nb_points;
    double lambda;
};

template <typename T>
std::vector<Eigen::VectorXd> PointGenerator::transform(T& DynamicalModel) const{
    std::vector<Eigen::VectorXd> Transformed_Points(0);
    for (int i = 0 ; i < nb_points ; ++i)
        Transformed_Points.push_back(DynamicalModel.transform(Points[i]));
    return Transformed_Points;
}


#endif
