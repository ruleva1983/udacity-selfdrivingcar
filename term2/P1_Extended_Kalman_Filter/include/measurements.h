#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Eigen/Dense"

class measurement{
public:
    measurement();
    virtual ~measurement();

    virtual Eigen::MatrixXd getR() const = 0;;
    virtual Eigen::MatrixXd getH(const Eigen::VectorXd& state) const = 0;
    virtual Eigen::VectorXd residual(const Eigen::VectorXd&, Eigen::VectorXd&) const = 0;
    

};

class Lidar: public measurement{
public:
    Lidar();
    virtual ~Lidar();
    
    Eigen::MatrixXd getR() const;
    Eigen::MatrixXd getH(const Eigen::VectorXd& state) const;
    Eigen::VectorXd residual(const Eigen::VectorXd&, Eigen::VectorXd&) const;
    

private:
    const double noise_lidar = 0.0225;
};

class Radar: public measurement{
public:
    Radar();
    virtual ~Radar();
    
    Eigen::MatrixXd getR() const;
    Eigen::MatrixXd getH(const Eigen::VectorXd& state) const;
    Eigen::VectorXd residual(const Eigen::VectorXd&, Eigen::VectorXd&) const;

    
private:
    const double noise_radar_rho = 0.09;
    const double noise_radar_theta = 0.0009;
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
