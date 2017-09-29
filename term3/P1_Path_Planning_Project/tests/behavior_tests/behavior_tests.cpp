#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <iostream>

#include "sensor_fusion.hpp"
#include "map.hpp"
#include "agent.hpp"
#include "behavior.hpp"


namespace unittest{
    
    TEST(ToolsTest, Behavior_struct)
    {
        Behavior b(false, 0, 35.0);
        ASSERT_EQ(b.change_lane , false);
        ASSERT_EQ(b.next_lane , 0);
        ASSERT_EQ(b.target_speed , 35);
    }
    
    
    TEST(ToolsTest, BehaviorPlanner_class)
    {
        BehaviorPlanner planner;
        
        std::vector<int> lanes = planner.get_available_lanes(0);
        ASSERT_EQ(lanes.size(), 2);
        
        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 19, 30, 10});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 0, 21, 45, 6});
        //vehicle_list.push_back(std::vector<double> {2, 0.0, 0.0, 0, 20.1, 30, 6});
        //vehicle_list.push_back(std::vector<double> {3, 0.0, 0.0, 0, 20.1, 70, 6});
        //vehicle_list.push_back(std::vector<double> {4, 0.0, 0.0, 0, 20.1, 30, 10});
        //vehicle_list.push_back(std::vector<double> {5, 0.0, 0.0, 0, 20.1, 45, 10});
    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6, 0.0, 0.0);
        
        double cost0 = planner.eval_lane_cost(0, map, vehicles, car);
        double cost1 = planner.eval_lane_cost(1, map, vehicles, car);
        double cost2 = planner.eval_lane_cost(2, map, vehicles, car);
        cout << cost0 <<  "   " << cost1  <<  "   " << cost2;
        
        
        
        
        
        
        
//         ASSERT_EQ(b.change_lane , false);
//         ASSERT_EQ(b.next_lane , 0);
//         ASSERT_EQ(b.target_speed , 35);
    }
    
    
   
}
