#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include "Eigen/Dense"

class DynamicalModel{
public:
    
    DynamicalModel();
    virtual ~DynamicalModel();
    
    virtual Eigen::VectorXd transform(Eigen::VectorXd& ) const = 0;
    virtual Eigen::MatrixXd getNoise() const = 0;

};


class CTRV: public DynamicalModel{
public:
    
    CTRV();
    virtual ~CTRV();
    
    Eigen::VectorXd transform(Eigen::VectorXd& , double) const;
    Eigen::MatrixXd getNoise() const;
    
private:
    int nb_state_variables = 5;
    double noise_a = 1.0, noise_psi = 1.0;
};

#endif
