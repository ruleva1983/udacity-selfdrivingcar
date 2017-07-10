#include "lidar.h"
#include "constants.h"


Lidar::Lidar() {}

Lidar::~Lidar() {}


Eigen::MatrixXd Lidar::getR(){
    Eigen::MatrixXd R(2,2);
    R << noise_lidar, 0.0,
         0.0,         noise_lidar;
    return R;
}

Eigen::MatrixXd Lidar::getH(){
    Eigen::MatrixXd H(2,4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;
    return H;
}
