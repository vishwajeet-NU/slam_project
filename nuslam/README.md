### "ME495 Sensing, Navigation, and Machine Learning" </br> ###

Author : Vishwajeet Karmarkar </br>

This package is used to implement EKF slam on a turtlebot </br>


To launch the contents : </br>

You can launch it in debug mode and normal mode </br>
In debug mode gazebo model state is used to populate sensor messages </br>
so that slam can be tested without worrying about data association </br>

to launch only landmark detector::</br>
<pre><code>roslaunch nuslam landmarks.launch robot:=-1 
</code></pre>
It looks as follows </br>

![image](images/landmark_detector.png)


robot argument is dependent on choice of platform </br>
this launch file also starts a keyboard teleop system which can be used to </br>
navigate the bot</br>


to launch slam 

<pre><code>roslaunch nuslam slam.launch robot:=-1 debug:=True </pre></code>

debug argument can be changed as needed

