#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "measurements.h"
#include <string>
#include <fstream>

using namespace std;

class Tools {
public:

  Tools();

  virtual ~Tools();

  Eigen::VectorXd CalculateRMSE(const std::vector< Eigen::VectorXd >& estimations, const std::vector< Eigen::VectorXd >& ground_truth);
  
  void EncodeLine(MeasurementPackage&, vector<Eigen::VectorXd>&, string& );
  
  void write_output(ofstream&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, double);
  
  static double NIS(Eigen::VectorXd& , Eigen::VectorXd& , Eigen::MatrixXd&);

};


#endif 
