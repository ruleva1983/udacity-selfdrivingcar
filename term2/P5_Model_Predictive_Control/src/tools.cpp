#include "tools.h"
#include <math.h>
#include "json.hpp"
#include "Eigen"

using json = nlohmann::json;


double tools::polyeval(const Eigen::VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd tools::polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double tools::deg2rad(double x) { return x * M_PI / 180; }
double tools::rad2deg(double x) { return x * 180 / M_PI; }
double tools::mphtoms(double x) { return x * 0.44704; }


Eigen::VectorXd tools::build_init_state(json j, const Eigen::VectorXd& coeffs){
    double cte = tools::polyeval(coeffs, 0);
    double epsi = -atan(coeffs[1]);
    
    Eigen::VectorXd state(6);
    state << 0, 0, 0, tools::mphtoms(j[1]["speed"]), cte, epsi;
    return state;
    
}


void tools::manage_input(json j, Eigen::VectorXd& x_traj, Eigen::VectorXd& y_traj){
    vector<double> ptsx = j[1]["ptsx"];
    vector<double> ptsy = j[1]["ptsy"];
    double px = j[1]["x"];
    double py = j[1]["y"];
    double psi = j[1]["psi"];
    double v = j[1]["speed"];
    v = mphtoms(v);

    x_traj.resize(ptsx.size());
    y_traj.resize(ptsy.size());
    for (int i = 0; i < ptsx.size(); i++) {
        x_traj[i] =  (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
        y_traj[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi) ;
    }
}

json tools::manage_output(double steer_value, double throttle_value, const Eigen::VectorXd& x_traj,
                   const Eigen::VectorXd& y_traj, const std::vector<double>& mpc_x_traj,
                   const std::vector<double>& mpc_y_traj){
    json msgJson;
    msgJson["steering_angle"] = -steer_value;
    msgJson["throttle"] = throttle_value;

    std::vector<double> vec_x_traj(x_traj.size());
    std::vector<double> vec_y_traj(x_traj.size());

    for (int i = 0; i < x_traj.size(); ++i){
        vec_x_traj[i] = x_traj[i];
        vec_y_traj[i] = y_traj[i];
        }

    msgJson["next_x"] = vec_x_traj;
    msgJson["next_y"] = vec_y_traj;
    msgJson["mpc_x"] = mpc_x_traj;
    msgJson["mpc_y"] = mpc_y_traj;
    return msgJson;    
}
