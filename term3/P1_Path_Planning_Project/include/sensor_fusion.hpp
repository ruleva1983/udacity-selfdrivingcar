#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include "map.hpp"
#include "agent.hpp"
//#include "behavior.hpp"

const double DistanceRadius2 = 35.0;


using namespace std;


struct VehicleData{
    

    VehicleData();
    VehicleData(const vector<double>& raw_data);
    
    double speed();
    
    
    int id;     
    double x, y, vx, vy, s, d;  
};


class Vehicles
{
public:
    
    /// \brief Parametrized Constructor
    /// \param raw_data A container of vectors each one holding the vehicle parameters as provided
    ///                 by the simulator. 
    Vehicles(const vector<vector<double>>& raw_data);
    
    /// \brief Provide a vector the copies of all the Vehicles in a given lane.
    /// \param lane the lane of interest
    /// \param map the map of the circuit
    /// \return The vector of the vehicles on the lane of interest.
    vector<VehicleData> veh_on_lane(int lane, const Map& map) const;
    
    
    
    /// \brief Provide the closest vehicle in front of the agent in the lane of interest.
    /// \param lane the lane of interest
    /// \param map the map of the circuit
    /// \param car the agent object
    /// \return The VehicleData object required. If this as id = -1 it means that no vehicle has been found
    VehicleData closest_vehicle_front(int lane, const Map& map, Agent& car) const;
    
    int size(){
        return vehicles.size();
    }
    
private:
    vector<VehicleData> vehicles;

};



VehicleData::VehicleData(){
    id = -1;
    x = 0;
    y = 0;
    s = 0;
    d = 0;
    vx = 0;
    vy = 0;
}

VehicleData::VehicleData(const std::vector<double>& raw_data):
        id(static_cast<int>(raw_data[0])),
        x(raw_data[1]),
        y(raw_data[2]),
        vx(raw_data[3]),
        vy(raw_data[4]),
        s(raw_data[5]),
        d(raw_data[6]){}


double VehicleData::speed(){
    return sqrt(vx*vx + vy*vy); //[m/s]
}

        
        
 Vehicles::Vehicles(const vector<vector<double>>& raw_data): vehicles(){
        for (int i = 0; i < raw_data.size(); ++i)
            vehicles.push_back(VehicleData(raw_data[i]));
    }

vector<VehicleData> Vehicles::veh_on_lane(int lane, const Map& map) const{
    vector<VehicleData> V;
    for (auto v : vehicles){
        if (map.current_lane(v.d) == lane)
            V.push_back(v);
    }
    return V;
}


VehicleData Vehicles::closest_vehicle_front(int lane, const Map& map, Agent& car) const
{
    vector<VehicleData> V = veh_on_lane(lane, map);
    
    VehicleData vv;
    //TODO Substitute to 50 DistanceRadius
    double min_distance = DistanceRadius2;
    for (auto v : V){
        double distance = map.distance(car.get_state().s, v.s);
        if (distance < 0 && abs(distance) < min_distance){
            min_distance = abs(distance);
            vv = v;
        }
    }
    return vv;
}





#endif 
