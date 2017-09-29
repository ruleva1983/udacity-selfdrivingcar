#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "agent.hpp"
#include "sensor_fusion.hpp"
#include "map"

struct FrenetState{
    
    double s, sdot, sdotdot = 0;
    double d, ddot = 0, ddotdot = 0;
};



class Generator{
public:
    Generator();
    void generate_trajectory(Agent& agent, const Map& map, const Vehicles& vehicles, int new_lane);
    
private:
    FrenetState start_state;
    FrenetState final_state;
    
};


#endif
