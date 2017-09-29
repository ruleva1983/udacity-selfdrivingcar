#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include<vector>
#include<limits>

#include "sensor_fusion.hpp"
#include "map.hpp"
#include "agent.hpp"
#include "utils.hpp"


const double max_speed = 50.0;
const double DistanceRadius = 50.0;
const double coeff_speed = 0.7;
const double coeff_distance = 0.25;
const double coeff_change = 1 - coeff_speed - coeff_distance;

struct Behavior{
    Behavior(bool b, int lane, float speed);
    bool change_lane;
    int next_lane;
    float target_speed;
};

class BehaviorPlanner
{
public:
    /// \brief Default Constructor.
    BehaviorPlanner();
    
    /// \brief Evaluates the best behavior given the map, vehicles and agent information as input.
    /// \param map The map object
    /// \param vehicles The vehicles sensor fusion object
    /// \param car The agent object
    /// \return The vector of the vehicles on the lane of interest.
    Behavior eval_behavior(const Map& map, const Vehicles& vehicles, Agent& car);

//TODO Reput private when we solve the issue with the unit tests    
//private:

    /// \brief Evaluates the cost of the lane of interest given the sensor, map and agent information. A part of the cost is associated to the speed used in the lane, another part to the traffic in the lane.
    /// \param lane The lane of interest
    /// \param map The map object
    /// \param vehicles The vehicles sensor fusion object
    /// \param car The agent object
    /// \return The lane cost
    double eval_lane_cost(int lane, const Map& map, const Vehicles& vehicles, Agent& car);

    /// \brief Evaluates the target speed if a given lane were to be taken as a behavioral target
    /// \param lane The lane of interest
    /// \param map The map object
    /// \param vehicles The vehicles sensor fusion object
    /// \param car The agent object
    /// \return The lane cost
    double eval_target_speed(int lane, const Map& map, const Vehicles& vehicles, Agent& car);
    
    
    /// \brief Returns the list of available lanes for the agent given its current lane. No double lane cross is allowed.
    /// \param current_lane The agent current lane
    /// \return The vector of available lanes
    vector<int> get_available_lanes(int current_lane);
    
    Behavior previous_behavior;
    
};


Behavior::Behavior(bool b, int lane, float speed) : change_lane(b), next_lane(lane), target_speed(speed)
{
}



BehaviorPlanner::BehaviorPlanner() : previous_behavior(false, -1, max_speed)
{
}

Behavior BehaviorPlanner::eval_behavior(const Map& map, const Vehicles& vehicles, Agent& car)
{
    int new_lane, current_lane = map.current_lane(car.get_state().d);
    vector<int> available_lanes = get_available_lanes(current_lane);
    
    cout <<  "Current lane: " << current_lane << endl;
    for (auto l : available_lanes)
        cout << "Available lane: " << l << endl;
    
    double lowest_cost = std::numeric_limits<double>::max();
    
    for (auto l : available_lanes){
        double lane_cost = eval_lane_cost(l, map, vehicles, car);
        cout << "Cost for lane " << l << " = " << lane_cost << endl; 
        if (lane_cost < lowest_cost){
            lowest_cost = lane_cost;
            new_lane = l;
        }
    }
    
    cout << "Lane chosen: " << new_lane << endl;
    
    double target_speed = eval_target_speed(new_lane, map, vehicles, car);
    
    cout << "Target Speed in the chosen lane: " <<  target_speed << endl;
    
    bool change_lane;
    if (new_lane == current_lane)
        change_lane = false;
    else
        change_lane = true;
    
    Behavior b(change_lane, new_lane, target_speed);
    previous_behavior = b;
    return b;
}


double BehaviorPlanner::eval_lane_cost(int lane, const Map& map, const Vehicles& vehicles, Agent& car)
{
    vector<VehicleData> lane_vehicles = vehicles.veh_on_lane(lane, map);
    
    // Speed related cost
    double min_speed = utils::mph_to_ms(max_speed);
    for (auto vehicle : lane_vehicles){
        
        double speed = utils::ms_to_mph(vehicle.speed());
        
        cout << "Lane: " << lane << " Id: " << vehicle.id << " speed: " << speed << endl;
        
        if (speed < min_speed){
            min_speed = speed;
        }
    }
    double v_cost = 1 - min_speed/max_speed;
    
    // Distance related cost
    double min_distance = DistanceRadius;
    for (auto vehicle : lane_vehicles){
        double distance = map.distance(car.get_state().s, vehicle.s);
        if (distance < 0 && distance < min_distance){
            min_distance = distance;
        }
    }
    double d_cost = 1 - min_distance/DistanceRadius;
    
    // Non lane change preferred cost
    
    double c_cost = abs(map.current_lane(car.get_state().d) - lane);
    
    double total_cost = coeff_speed*v_cost + coeff_change*c_cost + coeff_distance*d_cost;
    
    // Check for possible collisions in lane change
    
    return total_cost;
}

double BehaviorPlanner::eval_target_speed(int lane, const Map& map, const Vehicles& vehicles,
                                          Agent& car)
{
    VehicleData V = vehicles.closest_vehicle_front(lane, map, car);
    if (V.id == -1)
        return max_speed;
    else
        return utils::ms_to_mph(V.speed());
}



vector<int> BehaviorPlanner::get_available_lanes(int current_lane){
    if (current_lane == 1)
        return {0, 1, 2};
    if (current_lane == 0)
        return {0, 1};
    return {1, 2};
}



#endif
