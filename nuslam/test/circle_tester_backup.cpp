// tests.cpp
#include "ros/ros.h"
#include <gtest/gtest.h>
#include "nuslam/nuslam.hpp"
#include <vector>

class circle_tester
{
        public:
        ros::NodeHandle n;
        nuslam::test fake_laser;

        float x_center_reading;
        float y_center_reading;
        float radius_reading;        

        bool istrue = true;

        circle_tester(std::vector<float> x, std::vector<float> y)
        {

            ros::Publisher sensor_pub = n.advertise<nuslam::test>("test", 1, true);
            fake_laser.x_vals = x;
            fake_laser.y_vals = y;

            ros::Duration(0.2).sleep();
            

            sensor_pub.publish(fake_laser);

            ros::Subscriber landmark_sub = n.subscribe("/landmarks", 1, &circle_tester::callback, this);

            while(istrue)
            {
                std::cout<<"inside true \n";

                ros::spinOnce();
            }
                
        }

        void callback(const nuslam::turtle_map &land_in)
        {
            x_center_reading = land_in.x_center[0];
            y_center_reading =  land_in.y_center[0];
            radius_reading = land_in.radius[0];

            istrue = false;

        }

};

TEST(circle_test, readings_1)
	{
        std::vector<float> x_in = {1.0,2.0,5.0,7.0,9.0,3.0};
        std::vector<float> y_in = {7.0,6.0,8.0,7.0,5.0,7.0};

        double actual_center_x = 4.615482;
        double actual_center_y = 2.807354;
        double actual_radius =  4.8275;

        circle_tester test1(x_in,y_in);
        
        ASSERT_NEAR(test1.x_center_reading,actual_center_x,0.2);
        ASSERT_NEAR(test1.y_center_reading,actual_center_y,0.2);
        ASSERT_NEAR(test1.radius_reading,actual_radius,0.2);
 
        // ASSERT_NEAR(4.615482,actual_center_x,1e-4);
        // ASSERT_NEAR(2.807354,actual_center_y,1e-4);
        // ASSERT_NEAR(4.8275,actual_radius,1e-4);

       
       
    }	


int main(int argc, char * argv[])
{
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "circle_test");
    ros::NodeHandle n;

    return RUN_ALL_TESTS();
   

}
