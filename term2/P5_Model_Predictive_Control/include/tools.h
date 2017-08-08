#ifndef TOOLS_H
#define TOOLS_H


#include "Eigen"
#include <vector>
#include "tools.h"
#include "json.hpp"

using json = nlohmann::json;
using namespace std;

namespace tools{

double polyeval(const Eigen::VectorXd&, double);

Eigen::VectorXd polyfit(const Eigen::VectorXd&, const Eigen::VectorXd&, int);

void manage_input(json, Eigen::VectorXd&, Eigen::VectorXd&);

json manage_output(double, double, const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<double>&, const std::vector<double>&);

Eigen::VectorXd build_init_state(json, const Eigen::VectorXd&);

double deg2rad(double);
double rad2deg(double);
double mphtoms(double);
}
#endif
