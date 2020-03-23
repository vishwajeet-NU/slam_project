// tests.cpp
#include "ros/ros.h"
#include <gtest/gtest.h>
#include "nuslam/nuslam.hpp"
#include <vector>

TEST(circle_test, readings_1)
	{
        EKF test;

        std::vector<std::vector<float>> x;
        std::vector<std::vector<float>> y;
        
        std::vector<float> x_in = {1.0,2.0,5.0,7.0,9.0,3.0};
        std::vector<float> y_in = {7.0,6.0,8.0,7.0,5.0,7.0};

        x.push_back(x_in);
        y.push_back(y_in);
        double actual_center_x = 4.615482;
        double actual_center_y = 2.807354;
        double actual_radius =  4.8275;

        test.circle(x,y);

        ASSERT_NEAR(test.x_data[0],actual_center_x,1e-4);
        ASSERT_NEAR(test.y_data[0],actual_center_y,1e-4);

        ASSERT_NEAR(test.r_data[0],actual_radius,1e-4);
       
       
    }	


TEST(circle_test, readings_2)
	{
        EKF test;

        std::vector<std::vector<float>> x;
        std::vector<std::vector<float>> y;
        
        std::vector<float> x_in = {-1.0,-0.3,0.3,1.0};
        std::vector<float> y_in = {0.0,-0.06,0.1,0.0};

        x.push_back(x_in);
        y.push_back(y_in);
        double actual_center_x = 0.4908357;
        double actual_center_y = -22.15212;
        double actual_radius =  22.17979;

        test.circle(x,y);

        ASSERT_NEAR(test.x_data[0],actual_center_x,1e-4);
        ASSERT_NEAR(test.y_data[0],actual_center_y,1e-4);

        ASSERT_NEAR(test.r_data[0],actual_radius,1e-4);
       
       
    }	

int main(int argc, char * argv[])
{
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
   

}
