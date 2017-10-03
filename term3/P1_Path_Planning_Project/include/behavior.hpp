#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include<vector>
#include<limits>

#include "sensor_fusion.hpp"
#include "map.hpp"
#include "agent.hpp"
#include "utils.hpp"


const double MAX_SPEED = 45.0; //[mph]
const double DISTANCE_RADIUS = 25.0; //[m]
const double SPEED_DISTANCE_RADIUS = 40.0; //[m]
const double COLLISION_RADIUS = 5.0; //[m]
const double COEFF_V = 0.55;
const double COEFF_D = 0.4;
const double COEFF_C = 1 - COEFF_D - COEFF_V;
const double MAX_ACC = 9.7; //[ms^(-2)]
const double CHANGE_LANE_LIMIT = 0.05; //[m]

struct Behavior{
    Behavior(bool b, int lane, float speed);
    bool change_lane;
    int next_lane;
    float target_speed;
};

class BehaviorPlanner
{
public:
    /// \brief Default Constructor. Initializes members to default values 
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


BehaviorPlanner::BehaviorPlanner() : previous_behavior(false, -1, utils::mph_to_ms(MAX_SPEED))
{
}

Behavior BehaviorPlanner::eval_behavior(const Map& map, const Vehicles& vehicles, Agent& car)
{
    
    // If the vehicle is currently undergoing a lane change, do not change behavior until lane change is complete
    if (previous_behavior.change_lane){
        if (car.get_state().d - (previous_behavior.next_lane + 0.5)*LaneWidth > CHANGE_LANE_LIMIT)
            return previous_behavior;
    }

    int new_lane, current_lane = map.current_lane(car.get_state().d);
    vector<int> available_lanes = get_available_lanes(current_lane);
    
    double lowest_cost = std::numeric_limits<double>::max();
    
    for (auto l : available_lanes){
        double lane_cost = eval_lane_cost(l, map, vehicles, car);
        if (lane_cost < lowest_cost){
            lowest_cost = lane_cost;
            new_lane = l;
        }
    }
 
    double target_speed = eval_target_speed(new_lane, map, vehicles, car);
 
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
    double min_speed = utils::mph_to_ms(MAX_SPEED);
    for (auto vehicle : lane_vehicles){
        double distance = map.distance(car.get_state().s, vehicle.s);
        double speed = vehicle.speed();
        if (speed < min_speed && distance < 0 && abs(distance) < SPEED_DISTANCE_RADIUS){
            min_speed = speed;
        }
    }
    double v_cost = 1 - min_speed/utils::mph_to_ms(MAX_SPEED);
    
    double min_abs_distance = DISTANCE_RADIUS;
    for (auto vehicle : lane_vehicles){
        double distance = map.distance(car.get_state().s, vehicle.s);
        
        // AVOIDING COLLISION
        if ( abs(distance) < COLLISION_RADIUS){
            return 1.0;
        }
        if (distance < 0 && abs(distance) < min_abs_distance){
            min_abs_distance = abs(distance);
        }
    }
    double d_cost = 1 - min_abs_distance/DISTANCE_RADIUS;
    
    // Non lane change preferred cost
    double c_cost = abs(map.current_lane(car.get_state().d) - lane);
    
    double total_cost = COEFF_V*v_cost + COEFF_C*c_cost + COEFF_D*d_cost;
    return total_cost;
}

double BehaviorPlanner::eval_target_speed(int lane, const Map& map, const Vehicles& vehicles, Agent& car)
{
    VehicleData V = vehicles.closest_vehicle_front(lane, map, car);
    if (V.id == -1)
        return utils::mph_to_ms(MAX_SPEED);
    else
        return V.speed();
}

vector<int> BehaviorPlanner::get_available_lanes(int current_lane){
    if (current_lane == 1)
        return {0, 1, 2};
    if (current_lane == 0)
        return {0, 1};
    return {1, 2};
}



#endif
