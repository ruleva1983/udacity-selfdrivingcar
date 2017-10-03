#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <iostream>

#include "sensor_fusion.hpp"
#include "map.hpp"
#include "agent.hpp"
#include "behavior.hpp"
#include "utils.hpp"


//TODO Find better separation between the tests for the class objects
namespace unittest{
    
    TEST(ToolsTest, Behavior_struct)
    {
        Behavior b(false, 0, 35.0);
        ASSERT_EQ(b.change_lane , false);
        ASSERT_EQ(b.next_lane , 0);
        ASSERT_EQ(b.target_speed , 35);
    }
    
    TEST(ToolsTest, BehaviorPlanner_eval_lane_cost1)
    {
        BehaviorPlanner planner;
        
        std::vector<int> lanes = planner.get_available_lanes(0);
        ASSERT_EQ(lanes.size(), 2);
        
        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 19, 30, 10});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 0, 21, 45, 6});
    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6, 0.0, 0.0);
        
        double cost0 = planner.eval_lane_cost(0, map, vehicles, car);
        double cost1 = planner.eval_lane_cost(1, map, vehicles, car);
        double cost2 = planner.eval_lane_cost(2, map, vehicles, car);
        ASSERT_LT(cost0, cost1);
        ASSERT_LT(cost0, cost2);
        ASSERT_LT(cost1, cost2);

    }
    
    TEST(ToolsTest, BehaviorPlanner_eval_lane_cost2)
    {
        BehaviorPlanner planner;
        
        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 19, 70, 10});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 0, 21, 80, 6});
    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6, 0.0, 0.0);
        
        double cost0 = planner.eval_lane_cost(0, map, vehicles, car);
        double cost1 = planner.eval_lane_cost(1, map, vehicles, car);
        double cost2 = planner.eval_lane_cost(2, map, vehicles, car);
        ASSERT_LT(cost1, cost0);
        ASSERT_LT(cost1, cost2);

    }
   
    TEST(ToolsTest, BehaviorPlanner_eval_target_speed)
    {
        BehaviorPlanner planner;

        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 15, 40, 2});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 0, 10, 80, 2});
        
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 15, 100, 6});

    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6, 0.0, 0.0);
        
        double speed0 = planner.eval_target_speed(0, map, vehicles, car);
        double speed1 = planner.eval_target_speed(1, map, vehicles, car);
        
        ASSERT_EQ(speed0, 15);
        ASSERT_EQ(speed1, utils::mph_to_ms(MAX_SPEED));

    }
    
    TEST(ToolsTest, BehaviorPlanner_eval_behavior1)
    {
        BehaviorPlanner planner;
        
        std::vector<std::vector<double>> vehicle_list {};
        vehicle_list.push_back(std::vector<double> {0, 0.0, 0.0, 0, 19, 30, 10});
        vehicle_list.push_back(std::vector<double> {1, 0.0, 0.0, 0, 21, 45, 6});

    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 6, 0.0, 0.0);

        Behavior new_behavior = planner.eval_behavior(map, vehicles, car);
        
        ASSERT_EQ(new_behavior.change_lane, true);
        ASSERT_EQ(new_behavior.next_lane, 0);
        ASSERT_NEAR(new_behavior.target_speed, utils::mph_to_ms(MAX_SPEED), 0.0001);
    }
    
    TEST(ToolsTest, BehaviorPlanner_eval_behavior2)
    {
        BehaviorPlanner planner;
        
        std::vector<std::vector<double>> vehicle_list {};
    
        Vehicles vehicles(vehicle_list);
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        Agent car;
        car.set_state(0.0, 0.0, 15, 2, 0.0, 0.0);

        Behavior new_behavior = planner.eval_behavior(map, vehicles, car);
        
        ASSERT_EQ(new_behavior.change_lane, false);
        ASSERT_EQ(new_behavior.next_lane, 0);
        ASSERT_NEAR(new_behavior.target_speed, utils::mph_to_ms(MAX_SPEED), 0.0001);
    }
    
    
    
    
}
