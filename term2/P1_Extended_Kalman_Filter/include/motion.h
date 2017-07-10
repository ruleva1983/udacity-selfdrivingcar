#ifndef MOTION_H_
#define MOTION_H_

#include "Eigen/Dense"

class Motion{
public:
    
    Motion();
    virtual ~Motion();

    Eigen::MatrixXd getF(double) const;
    Eigen::MatrixXd getQ(double) const;

};


#endif
