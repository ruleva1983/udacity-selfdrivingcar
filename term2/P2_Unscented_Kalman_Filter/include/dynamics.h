#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include "Eigen/Dense"

class DynamicalModel{
public:
    
    DynamicalModel();
    virtual ~DynamicalModel();
    
    virtual Eigen::VectorXd transform(Eigen::VectorXd& , double) const = 0;
    virtual Eigen::MatrixXd getNoise() const = 0;
    
    virtual void augmentState(Eigen::VectorXd&, Eigen::MatrixXd&) const = 0;

};


class CTRV: public DynamicalModel{
public:
    
    CTRV();
    virtual ~CTRV();
    
    Eigen::VectorXd transform(Eigen::VectorXd& , double) const;
    Eigen::MatrixXd getNoise() const;
    void augmentState(Eigen::VectorXd&, Eigen::MatrixXd&) const;
    
    int nb_state_variables = 5; 
    
private:
    int nb_noise_comp = 2;
    
};

#endif
