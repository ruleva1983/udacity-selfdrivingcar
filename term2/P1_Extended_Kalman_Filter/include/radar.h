#ifndef RADAR_H_
#define RADAR_H_

#include "Eigen/Dense"


class Radar{
public:
    Radar();
    virtual ~Radar();
    
    Eigen::MatrixXd getR();
    Eigen::MatrixXd getH(const Eigen::VectorXd& state);
    
};


#endif
