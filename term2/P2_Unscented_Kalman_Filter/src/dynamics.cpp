#include <cassert>

#include "dynamics.h"

DynamicalModel::DynamicalModel(){}
DynamicalModel::~DynamicalModel(){}
Eigen::MatrixXd DynamicalModel::getNoise() const{};

CTRV::CTRV(){}
CTRV::~CTRV(){}

// Takes as a input an augmented state and propagates it in time according to the model and the noise
Eigen::VectorXd CTRV::transform(Eigen::VectorXd& Input_State, double dt) const
{
    assert (Input_State.size() == 7);
    
    double x = Input_State(0), y = Input_State(1);
    double v = Input_State(2);
    double psi = Input_State(3), psid = Input_State(4);
    double nu_a = Input_State(5) , nu_psidd = Input_State(6); 
    
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

    Eigen::VectorXd Output_State(5);
    Output_State << xp, yp, vp, psip, psidp;
    return Output_State;
}

Eigen::MatrixXd CTRV::getNoise() const
{
    Eigen::MatrixXd NoiseMatrix(2,2);
    NoiseMatrix << noise_a, 0.0,
                   0.0,     noise_psi;
    return NoiseMatrix;
}
