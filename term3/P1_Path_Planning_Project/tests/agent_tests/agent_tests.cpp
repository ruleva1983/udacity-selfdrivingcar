#include "agent.hpp"
#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <iostream>


namespace unittest{
    
    TEST(ToolsTest, State_struct)
    {
        State state;
        ASSERT_EQ(state.x, 0);
        ASSERT_EQ(state.y, 0);
        ASSERT_EQ(state.s, 0);
        ASSERT_EQ(state.d, 0);
        ASSERT_EQ(state.v, 0);
        ASSERT_EQ(state.yaw, 0);
        
        State state2 (1);
        ASSERT_EQ(state2.x, 1);
        ASSERT_EQ(state2.y, 0);
        ASSERT_EQ(state2.s, 0);
        ASSERT_EQ(state2.d, 0);
        ASSERT_EQ(state2.v, 0);
        ASSERT_EQ(state2.yaw, 0);
    }
 
    TEST(ToolsTest, Agent_Class)
    {

        Agent car;
        car.set_state(1,0,0,1,0,0);
        ASSERT_EQ(car.get_state().x, 1);
        ASSERT_EQ(car.get_state().y, 0);
        ASSERT_EQ(car.get_state().s, 0);
        ASSERT_EQ(car.get_state().d, 1);
        ASSERT_EQ(car.get_state().v, 0);
        ASSERT_EQ(car.get_state().yaw, 0);
 
    }

}
