Author: Vishwajeet Karmarkar </br>
vishwajeetkarmarkar2021@u.northwestern.edu </br>

This package has a plugin that helps </br>
create a simulated gazebo environemnt </br>

To launch the contents : 
roslaunch nuturtle_gazebo gazebo_waypoints.launch robot:=-1


to start moving :

rosservice call /start "direction: 1" 

to stop moving 

rosservice call /stop
