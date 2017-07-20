#include <cassert>

#include "constants.h"
#include "dynamics.h"
#include <iostream>

using namespace std;


DynamicalModel::DynamicalModel(){}
DynamicalModel::~DynamicalModel(){}
Eigen::MatrixXd DynamicalModel::getNoise() const{}
void DynamicalModel::augmentState(Eigen::VectorXd&, Eigen::MatrixXd&) const {}
Eigen::VectorXd DynamicalModel::transform(Eigen::VectorXd&, double) const
{}


CTRV::CTRV(){}
CTRV::~CTRV(){}


Eigen::VectorXd CTRV::transform(Eigen::VectorXd& Input_State, double dt) const
{
    assert (Input_State.size() == nb_state_variables + nb_noise_comp);
    
    double x = Input_State(0), y = Input_State(1);
    double v = Input_State(2);
    double psi = Input_State(3), psid = Input_State(4);
    double nu_a = Input_State(5), nu_psidd = Input_State(6);
    
    double xp, yp;
    
    if (fabs(psid) > 0.001) {
        xp = x + v/psid * ( sin (psi + psid*dt) - sin(psi));
        yp = y + v/psid * ( cos(psi) - cos(psi + psid*dt) );
    }
    else {
        xp = x + v*dt*cos(psi);
        yp = y + v*dt*sin(psi);
    }

    double vp = v;
    double psip = psi + psid*dt;
    double psidp = psid;
    
    xp += 0.5*nu_a*dt*dt * cos(psi);
    yp += 0.5*nu_a*dt*dt * sin(psi);
    vp += nu_a*dt;

    psip += 0.5*nu_psidd*dt*dt;
    psidp += nu_psidd*dt;

    Eigen::VectorXd Output_State(nb_state_variables);
    Output_State << xp, yp, vp, psip, psidp;

    return Output_State;
}

Eigen::MatrixXd CTRV::getNoise() const
{
    Eigen::MatrixXd NoiseMatrix(nb_noise_comp, nb_noise_comp);
    NoiseMatrix << noise_a, 0.0,
                   0.0,     noise_psi;
    return NoiseMatrix;
}

void CTRV::augmentState(Eigen::VectorXd& X_, Eigen::MatrixXd& P_) const
{
    int nb_aug = X_.size() + nb_noise_comp;  
    
    Eigen::VectorXd X_augmented = Eigen::VectorXd::Zero(nb_aug);
    for (int i = 0 ; i < X_.size() ; ++i)
        X_augmented[i] = X_[i];
    
    Eigen::MatrixXd P_augmented = Eigen::MatrixXd::Zero(nb_aug,nb_aug);
    P_augmented.block(0, 0, nb_state_variables ,nb_state_variables) = P_;  
    P_augmented.block(nb_state_variables, nb_state_variables, nb_noise_comp, nb_noise_comp) = getNoise();
    
    X_ = X_augmented;
    P_ = P_augmented;
}
