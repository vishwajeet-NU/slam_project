### "ME495 Sensing, Navigation, and Machine Learning" </br> ###
Author: Vishwajeet Karmarkar </br>
vishwajeetkarmarkar2021@u.northwestern.edu </br>

This package has a plugin that helps </br>
create a simulated gazebo environemnt </br>

To launch the contents : 
<pre><code>roslaunch nuturtle_gazebo gazebo_waypoints.launch robot:=-1
<code></pre>

to start moving :

<pre><code> rosservice call /start "direction: 1" 
<code></pre>
to stop moving 

rosservice call /stop
