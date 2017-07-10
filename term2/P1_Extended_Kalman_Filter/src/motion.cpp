#include "motion.h"
#include "constants.h"

Motion::Motion() {}

Motion::~Motion() {}

Eigen::MatrixXd Motion::getF(double dt) const{
    Eigen::MatrixXd F(4, 4);
     F << 1.0,   0.0,    dt,    0.0,
          0.0,   1.0,    0.0,   dt,
          0.0,   0.0,    1.0,   0.0,
          0.0,   0.0,    0.0,   1.0;
    return F;
}


Eigen::MatrixXd Motion::getQ(double dt) const{
    Eigen::MatrixXd Q(4, 4);
    
    double dt2 = std::pow(dt, 2);
    double dt3 = std::pow(dt, 3)/2.0;
    double dt4 = std::pow(dt, 4)/4.0;
    
     Q << dt4*noise_ax, 0,            dt3*noise_ax, 0,
          0,            dt4*noise_ay, 0,            dt3*noise_ay,
          dt3*noise_ax, 0,            dt2*noise_ax, 0,
          0,            dt3*noise_ay, 0,            dt2*noise_ay;
    return Q;
}
