"ME495 Sensing, Navigation, and Machine Learning"
Author : Vishwajeet Karmarkar </br>

This package is used to drive a turtle bot along a pentagonal path </br>
The path is first visualised in rviz by creating simulated encoder data </br>

To launch the contents : 
roslaunch nuturtle_robot follow_waypoints.launch robot:=(number of the bot to start on)

to start moving :

rosservice call /start "direction: 1" 

to stop moving 

rosservice call /stop