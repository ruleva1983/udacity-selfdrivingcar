#ifndef AGENT_H
#define AGENT_H

#include <iostream>

#include "utils.hpp"

struct State{

    State(double _x = 0, double _y = 0, double _s = 0,
          double _d = 0, double _speed = 0, double _theta = 0);
            
    double x, y;
    double s, d;
    double v, yaw;
};


struct FrenetState{

    FrenetState(double _s = 0, double _sdot = 0, double _sddot = 0, double _d = 0, double _ddot = 0, double _dddot = 0);
    
    void print(){
        std::cout << " s = "  << s << ", s_dot : " << s_dot << ", s_ddot " << s_ddot << 
                     ", d = "  << d << ", d_dot : " << d_dot << ", d_ddot " << d_ddot << std::endl;
    }
    double s, s_dot, s_ddot;
    double d, d_dot, d_ddot;

};



class Agent{
public:
    //TODO Build a costructor that direclty sets the state of the vehicle
    Agent();
    void set_state(double x, double y, double s,
          double d, double speed, double theta);
    
    State get_state();
    
    
private:
    State current_state;

};



State::State(double _x, double _y, double _s, double _d, double _speed, double _theta)
             : x(_x), y(_y), s(_s), d(_d), v(_speed), yaw(_theta)
             {
             }
             
             
FrenetState::FrenetState(double _s, double _sdot, double _sddot, double _d, double _ddot, double _dddot)
             : s(_s), s_dot(_sdot), s_ddot(_sddot), d(_d), d_dot(_ddot), d_ddot(_dddot)
             {
             }
             
Agent::Agent()
{
}

void Agent::set_state(double x, double y, double s, double d, double speed, double theta)
{
    current_state.x = x;
    current_state.y = y;
    current_state.s = s;
    current_state.d = d;
    current_state.v = utils::mph_to_ms(speed);
    current_state.yaw = theta;
}

State Agent::get_state()
{
    return current_state;
}







#endif
