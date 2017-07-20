#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Eigen/Dense"
#include "sigmapoints.h"

class measurement{
public:
    measurement();
    virtual ~measurement();

    virtual void initialize(const Eigen::VectorXd&, Eigen::VectorXd&, Eigen::MatrixXd&) const = 0;
    
    virtual Eigen::MatrixXd transform(Eigen::MatrixXd&) const = 0;
    
    void evalState(PointGenerator&, Eigen::VectorXd&, Eigen::MatrixXd&) const;
    virtual void AddNoise(Eigen::MatrixXd&) const = 0;
    
};

class Lidar: public measurement{
public:
    Lidar();
    virtual ~Lidar();
    
    void initialize(const Eigen::VectorXd& , Eigen::VectorXd& , Eigen::MatrixXd& ) const;
    
    Eigen::MatrixXd transform(Eigen::MatrixXd&) const;
    void AddNoise(Eigen::MatrixXd&) const;
 
private:
    const double nb_meas = 2;
    Eigen::VectorXd X_initialize;
    Eigen::MatrixXd P_initialize;
};


class Radar: public measurement{
public:
    Radar();
    virtual ~Radar();
    
    void initialize(const Eigen::VectorXd& , Eigen::VectorXd& , Eigen::MatrixXd& ) const;
    
    Eigen::MatrixXd transform(Eigen::MatrixXd&) const;
    void AddNoise(Eigen::MatrixXd&) const;
    
private:
    const double nb_meas = 3;
    Eigen::VectorXd X_initialize;
    Eigen::MatrixXd P_initialize;
};



class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};



#endif
