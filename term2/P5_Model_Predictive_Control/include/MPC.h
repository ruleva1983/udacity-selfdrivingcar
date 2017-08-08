#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen"

using namespace std;

class MPC {
 public:
  MPC();
  virtual ~MPC();

  vector<double> Solve(Eigen::VectorXd, Eigen::VectorXd, std::vector<double>&, std::vector<double>&);
};

#endif 
