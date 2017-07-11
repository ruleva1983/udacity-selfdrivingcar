#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include "Eigen/Dense"

class DynamicalModel{
public:
    
    DynamicalModel();
    virtual ~DynamicalModel();
    
    Eigen::MatrixXd getF(double) const;
    Eigen::MatrixXd getQ(double) const;

private:
    double noise_ax = 9.0;
    double noise_ay = 9.0;
    
};


#endif
