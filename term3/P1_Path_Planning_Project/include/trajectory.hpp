#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

#include "agent.hpp"
#include "sensor_fusion.hpp"
#include "map"
#include "behavior.hpp"
#include "utils.hpp"

const double DeltaT = 0.02;
const double MAX_T = 2;
const double MIN_T = 0.1;
int NB_TRAJ_POINTS = static_cast<int> (MAX_T/DeltaT);
int NB_PREV_POINTS = static_cast<int> (MIN_T/DeltaT);

using namespace std;


class Generator{
public:
    /// \brief Constructor. Initializes the previous_s and previous_d member to be empty vectors.
    Generator(){
        previous_s.resize(0);
        previous_d.resize(0);
    }
    
    
    /// \brief Transcribes the chosen trajectory points on the variables X, and Y, given the behavior imposed.
    /// \param behavior A behavior object passed by the behavioral planner
    /// \param car The agent object containing its state.
    /// \param map The map object
    /// \param previous_x The previously passed trajectory x points minus those points that have been consumed.
    /// \param previous_y The previously passed trajectory y points minus those points that have been consumed.
    /// \param X The newly computed trajectory x points.
    /// \param Y The newly computed trajectory y points.
    void generate_trajectory(Behavior& behavior, Agent& car, const Map& map,
                             vector<double>& previous_x, vector<double>& previous_y, vector<double>& X, vector<double>& Y);

private:
    
    /// \brief Evaluates the trajectory in Frenet coodinates given initial and final conditions and writes them on the X and Y variables
    /// \param map The map object
    /// \param init_state The initial state in Frenet frame
    /// \param target_state The target state in Frenet frame
    /// \param nb_new_points The number of new trajectory points to add to the trajectory
    /// \param X The newly computed trajectory x points.
    /// \param Y The newly computed trajectory y points.
    void eval_trajectory(const Map& map, FrenetState& init_state, FrenetState& target_state, int nb_new_points, vector<double>& X, vector<double>& Y);
    
    
    /// \brief Evaluates the coefficients of the second order polynomial for the s Frenet coordinate given initial position and velocity and final velocity.
    /// \param s0 Initial longitudinal position
    /// \param s0_dot Initial longitudinal velocity
    /// \param sf_dot Final longitudinal velocity
    /// \param T Integration time
    /// \param coeffs An empty vector over which the coefficients values will be written.
    void sTrajectory(double s0, double s0_dot, double sf_dot, double T, std::vector<double>& coeffs);
    
    /// \brief Evaluates the coefficients of the fifth order polynomial for the d Frenet coordinate given initial and final boundary conditions.
    /// \param d0 Initial lateral position
    /// \param d0_dot Initial lateral velocity
    /// \param d0_ddot Initial lateral acceleration
    /// \param df Final lateral position
    /// \param df_dot Final lateral position
    /// \param df_ddot Initial lateral velocity
    /// \param T Integration time
    /// \param coeffs An empty vector over which the coefficients values will be written.
    void dTrajectory(double d0, double d0_dot, double d0_ddot,
                     double df, double df_dot, double df_ddot, 
                     double T, std::vector<double>& coeffs);
                     
    /// \brief Evaluates the polynomial value given the coefficients and the independent variable x
    /// \param x Independent variable value
    /// \param coeffs The coefficients of the polynomial
    /// \return The function value
    double evaluate(double x, const std::vector<double>& coeffs);
    
    
    std::vector<double> previous_s;
    std::vector<double> previous_d;

};


// Member functions implementations


void Generator::generate_trajectory(Behavior& behavior, Agent& car, const Map& map,
                             vector<double>& previous_x, vector<double>& previous_y, vector<double>& X, vector<double>& Y){

        int nb_points_used = NB_TRAJ_POINTS - previous_x.size();
        
        if (nb_points_used < NB_TRAJ_POINTS){
            std::vector<double> new_s (previous_s.begin() + nb_points_used, previous_s.end());
            previous_s = new_s;
            std::vector<double> new_d (previous_d.begin() + nb_points_used, previous_d.end());
            previous_d = new_d;
        }
        
        int x_size = previous_x.size();
        int n_points_keep = std::min(x_size, NB_PREV_POINTS);
        previous_s.resize(n_points_keep);
        previous_d.resize(n_points_keep);

        for (int i = 0; i < n_points_keep; ++i){
            X.push_back(previous_x[i]);
            Y.push_back(previous_y[i]);
        }

        FrenetState initial_frenet_state;
        
        //TODO Estimation of the speed and acceleration using forward finite difference, maybe not accurate enough
        if (previous_x.size() < 2){
            initial_frenet_state.s = car.get_state().s;
            initial_frenet_state.d = car.get_state().d;
        }
        else{
            int last = previous_s.size() - 1;
            initial_frenet_state.s = previous_s[last];
            initial_frenet_state.s_dot = (previous_s[last]-previous_s[last-1])/DeltaT;
            initial_frenet_state.s_ddot = (previous_s[last]- 2*previous_s[last-1] + previous_s[last-2])/std::pow(DeltaT,2);

            initial_frenet_state.d = previous_d[last];
            initial_frenet_state.d_dot = (previous_d[last]-previous_d[last-1])/DeltaT;
            initial_frenet_state.d_ddot = (previous_d[last]- 2*previous_d[last-1] + previous_d[last-2])/std::pow(DeltaT,2);
        }
        

        const int nb_new_points = NB_TRAJ_POINTS - n_points_keep;
        FrenetState final_frenet_state;
        
        if (!behavior.change_lane){
            double current_lane = static_cast<double>(map.current_lane(car.get_state().d));
            final_frenet_state.d = (current_lane + 0.5) * LaneWidth;
            final_frenet_state.s_dot = behavior.target_speed;
        }
        else{
            final_frenet_state.d = (behavior.next_lane + 0.5) * LaneWidth;
            final_frenet_state.s_dot = utils::mph_to_ms(MAX_SPEED);
        }
        
        eval_trajectory(map, initial_frenet_state, final_frenet_state, nb_new_points, X, Y);
        
    }


void Generator::eval_trajectory(const Map& map, FrenetState& init_state, FrenetState& target_state, int nb_new_points, vector<double>& X, vector<double>& Y){

        double Tf = nb_new_points * DeltaT;

        std::vector<double> coeffs_s;
        sTrajectory(init_state.s, init_state.s_dot, target_state.s_dot, Tf, coeffs_s);

        std::vector<double> coeffs_d;
        dTrajectory(init_state.d, init_state.d_dot, init_state.d_ddot, target_state.d, 0.0, 0.0, Tf, coeffs_d);

        
        for (int i = 0; i < nb_new_points; ++i){    
            double t = (i + 1) * DeltaT;
            double d = evaluate(t, coeffs_d);
            double s = std::fmod(evaluate(t, coeffs_s), MaxS);
            
            previous_s.push_back(s);
            previous_d.push_back(d);

            X.push_back(map.frenet_to_xy(s, d).first);
            Y.push_back(map.frenet_to_xy(s, d).second);
        }

    }



void Generator::sTrajectory(double s0, double s0_dot, double sf_dot, double T, std::vector<double>& coeffs){
        coeffs.push_back(s0);
        coeffs.push_back(s0_dot);
        coeffs.push_back((sf_dot - s0_dot) / (2*T));
    }


void Generator::dTrajectory(double d0, double d0_dot, double d0_ddot,
                     double df, double df_dot, double df_ddot, 
                     double T, std::vector<double>& coeffs){

        Eigen::Matrix3d A;
        A << pow(T,3),    pow(T,4),    pow(T,5),
             3*pow(T,2),  4*pow(T,3),  5*pow(T,4),
             6*T,         12*pow(T,2), 20*pow(T,3);

        Eigen::Vector3d b;
        b << df  - d0 - d0_dot*T - d0_ddot*T*T/2.0,
             df_dot  - d0_dot  -  d0_ddot*T,
             df_ddot - d0_ddot;

        Eigen::Vector3d result = A.inverse() * b;
        coeffs.push_back(d0);
        coeffs.push_back(d0_dot);
        coeffs.push_back(d0_ddot/2.0);
        coeffs.push_back(result[0]);
        coeffs.push_back(result[1]);
        coeffs.push_back(result[2]);
    }
                     
double Generator::evaluate(double x, const std::vector<double>& coeffs){
        double sum = coeffs[0];
        for (int i = 1 ; i < coeffs.size() ; ++i)
            sum += coeffs[i]*std::pow(x, i);
        return sum;
    }





#endif
