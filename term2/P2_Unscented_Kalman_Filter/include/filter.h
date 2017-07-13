#ifndef FILTER_H_
#define FILTER_H_

#include "sigmapoints.h"
#include "Eigen/Dense"

#include <iostream>

using namespace std;



class BayesianFilter{
public:
    BayesianFilter();
    virtual ~BayesianFilter();
    
    //virtual void predict(Eigen::VectorXd&, Eigen::MatrixXd&)  const = 0;
    
};



class UFK : public BayesianFilter{
public:
    UFK();
    virtual ~UFK();
    
    template <typename DynamicalType>
    void predict(Eigen::VectorXd&, Eigen::MatrixXd&, DynamicalType, double);
    
    template <typename MeasurementType>
    void update(MeasurementType& , Eigen::VectorXd&, Eigen::MatrixXd&);
    
    void apply_kalman(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::MatrixXd&, Eigen::VectorXd&, Eigen::MatrixXd&,
                      Eigen::VectorXd&, Eigen::MatrixXd&);
    
private:
    PointGenerator Generator;
};



template <typename MeasurementType>
void UFK::update(MeasurementType& measDevice, Eigen::VectorXd& X_, Eigen::MatrixXd& P_)
{
    Eigen::MatrixXd sigmaPoints = Generator.Points_pred;
    cout << "Passing sigma Points through measurement..." << endl;
    Generator.Points_meas = measDevice.transform(sigmaPoints);
    
    cout << "Evaluate state after measurement..." << endl;
    measDevice.evalState(Generator.Points_meas, X_, P_);
    
    cout << "Adding measurement noise..." << endl;
    measDevice.AddNoise(P_);
}

template <typename DynamicalType>
void UFK::predict(Eigen::VectorXd& X_, Eigen::MatrixXd& P_, DynamicalType dynamicalSystem, double dt)
{
    cout << "Augmenting State..." << endl;
    dynamicalSystem.augmentState(X_, P_);
    
    cout << "Generating Sigma Points..." << endl;
    Generator.generate(X_, P_);

    cout << "Propagating Sigma Points..." << endl;

    Generator.transform<DynamicalType>(dynamicalSystem, dt);
    
    cout << "Updating State Mean and Covariance..." << endl;
    Generator.evalState(X_, P_);
}


#endif
