#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <iostream>

#include "sensor_fusion.hpp"
#include "map.hpp"
#include "agent.hpp"


namespace unittest{
    
    TEST(ToolsTest, VehicleData_struct)
    {
        VehicleData vehicle;
    
        ASSERT_EQ(vehicle.id, -1);
        ASSERT_EQ(vehicle.x, 0.0);
        ASSERT_EQ(vehicle.y, 0.0);
        ASSERT_EQ(vehicle.s, 0.0);
        ASSERT_EQ(vehicle.d, 0.0);
        ASSERT_EQ(vehicle.vx, 0.0);
        ASSERT_EQ(vehicle.vy, 0.0);
        
        ASSERT_EQ(vehicle.speed(), 0.0);
        
        double x = 1.1, y = 2.1, vx = 1.0, vy = 9.12, s = 0.1, d = 0.32;
        std::vector<double> raw_data {0, x, y, vx, vy, s, d};
        
        VehicleData vehicle2(raw_data);
        
        ASSERT_EQ(vehicle2.id, 0);
        ASSERT_EQ(vehicle2.x, x);
        ASSERT_EQ(vehicle2.y, y);
        ASSERT_EQ(vehicle2.s, s);
        ASSERT_EQ(vehicle2.d, d);
        ASSERT_EQ(vehicle2.vx, vx);
        ASSERT_EQ(vehicle2.vy, vy);
        
    }
    
    
    TEST(ToolsTest, Vehicles_class)
    {
        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 10.1, 20.1, 10, 2});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 10.1, 20.1, 40, 2});
        vehicle_list.push_back(std::vector<double> {2, 0.0, 0.0, 10.1, 20.1, 30, 6});
        vehicle_list.push_back(std::vector<double> {3, 0.0, 0.0, 10.1, 20.1, 70, 6});
        vehicle_list.push_back(std::vector<double> {4, 0.0, 0.0, 10.1, 20.1, 30, 10});
        vehicle_list.push_back(std::vector<double> {5, 0.0, 0.0, 10.1, 20.1, 45, 10});
    
        Vehicles vehicles(vehicle_list);
        
        ASSERT_EQ(vehicles.size(), vehicle_list.size());
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        int lane0_veh = vehicles.veh_on_lane(0, map).size();
        int lane1_veh = vehicles.veh_on_lane(1, map).size();
        int lane2_veh = vehicles.veh_on_lane(2, map).size();
        
        ASSERT_EQ(lane0_veh, 2);
        ASSERT_EQ(lane1_veh, 2);
        ASSERT_EQ(lane2_veh, 2);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6.1, 0.0, 0.0);
        
        VehicleData vehicle1 = vehicles.closest_vehicle_front(1, map, car);
        ASSERT_EQ(vehicle1.id, 2);
        
        VehicleData vehicle2 = vehicles.closest_vehicle_front(0, map, car);
        ASSERT_EQ(vehicle2.id, 1);
        
        VehicleData vehicle3 = vehicles.closest_vehicle_front(2, map, car);
        ASSERT_EQ(vehicle3.id, 4);
        
        car.set_state(0.0, 0.0, 80, 6.1, 0.0, 0.0);
        VehicleData vehicle4 = vehicles.closest_vehicle_front(0, map, car);
        ASSERT_EQ(vehicle4.id, -1);
        
    }
   
}
