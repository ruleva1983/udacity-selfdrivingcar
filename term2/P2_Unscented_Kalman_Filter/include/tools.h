#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "measurements.h"
#include "json.hpp"
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using json = nlohmann::json;

class Tools {
public:

  Tools();

  virtual ~Tools();

  Eigen::VectorXd CalculateRMSE(const std::vector< Eigen::VectorXd >& estimations, const std::vector< Eigen::VectorXd >& ground_truth);
  
  void EncodeLine(MeasurementPackage&, vector<VectorXd>&, string& );
  
  void write_output(ofstream&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&);

  
};


#endif 
