#include "map.hpp"
#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <iostream>



namespace unittest{
    
    TEST(ToolsTest, Waypoints_Struct)
    {
        string file_name = "../../../tests/test_data/test_map.csv";
        Waypoints wpts(file_name);
    
        ASSERT_NEAR (1.2, wpts.x[0], 0.0001);
        ASSERT_EQ(wpts.x.size(), 2);
        ASSERT_EQ(wpts.y.size(), 2);
        
        file_name = "../../../data/highway_map.csv";
        Waypoints wpts2(file_name);
    
        ASSERT_NEAR (1.2, wpts.x[0], 0.0001);
        ASSERT_EQ(wpts.x.size(), 2);
        ASSERT_EQ(wpts.y.size(), 2);
    
    }
    
    TEST(ToolsTest, Map_Class)
    {
        
        string file_name = "../../../data/highway_map.csv";
        Map map(file_name);
        
        //TODO Check map.size() function
        //cout << map.size();
        //ASSERT_EQ(map.size(), 182);

        
    }
    
    
    

}
