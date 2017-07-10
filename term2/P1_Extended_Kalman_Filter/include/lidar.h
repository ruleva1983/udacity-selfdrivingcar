#ifndef LIDAR_H_
#define LIDAR_H_

#include "Eigen/Dense"


class Lidar{
public:
    Lidar();
    virtual ~Lidar();
    
    Eigen::MatrixXd getR();
    Eigen::MatrixXd getH();
    
};


#endif
