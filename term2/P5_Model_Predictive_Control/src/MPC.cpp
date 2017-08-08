#include "MPC.h"
#include "tools.h"
#include "constants.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen"

using CppAD::AD;

int x_start = 0;
int y_start = N;
int psi_start = 2*N;
int v_start = 3*N;
int cte_start = 4*N;
int epsi_start = 5*N;
int delta_start = 6*N;
int a_start = 7*N - 1;


class FG_eval {
    
public:
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
    void operator()(ADvector& fg, const ADvector& vars) {
      
        fg[0] = 0;
        
        for (int t = 0; t < N; t++) {
            fg[0] += cte_factor*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += epsi_factor*CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += v_factor*CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        for (int t = 0; t < N - 1; t++) {
            fg[0] += delta_factor*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += a_factor*CppAD::pow(vars[a_start + t], 2);
        }

        for (int t = 0; t < N - 2; t++) {
            fg[0] += delta_diff_factor*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += a_diff_factor*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
      
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        for (int t = 1; t < N; t++) {

            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            //Correction for latency
            x0 = x0 + v0*CppAD::cos(psi0)*latency;
            y0 = y0 + v0*CppAD::sin(psi0)*latency;
            psi0 = psi0 + v0/Lf *delta0*latency;
            v0 = v0 + a0*latency;
            
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];
            
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 + coeffs[3] * x0*x0*x0;
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(psi0 - psides0) * dt);
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};


MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, std::vector<double>& xvals, std::vector<double>& yvals) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  int n_vars = N * 6 + (N - 1) * 2;
  int n_constraints = N * 6;

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
    vars[i] = 0.0;

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
 
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  FG_eval fg_eval(coeffs);

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    xvals.resize(N);
    yvals.resize(N);

    for (int i = 0; i < N; ++i){
        xvals[i] = solution.x[x_start + i];
        yvals[i] = solution.x[y_start + i];
    }
  
  return {solution.x[delta_start],   solution.x[a_start]};
}
