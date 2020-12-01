"ME495 Sensing, Navigation, and Machine Learning"
Author : Vishwajeet Karmarkar </br>

This repository houses code for a SLAM class at Northwestern Univerisity, January 2020. 
```bash
├── Homework2_Answers.md
├── nuslam
├── nuturtle_description
├── nuturtle_gazebo
├── nuturtle_robot
├── README.md
├── rigid2d
├── SLAM_RESULTS.md
└── tsim
```

The structure is shown above. <br>
1) rigid2d: A c++ library for modelling a diff drive robot and kinematics. Also houses helper functions.<br>
2) nuturtle_description: Urdf files, launch files for starting Rviz visualisation <br>
3) nuturtle_robot: Robot control code 
4) nuturtle_gazebo: Xacro files along with sensor plugins for Gazebo simulation <br>
5) tsim : Waypoint follower for turtlebot <br>
6) nuslam: Slam code 

To run: 

``` 
mkdir src 
cd src 
```

``` 
git clone https://github.com/vishwajeet-NU/slam_project.git
```
```
catkin_make
```

To launch slam <br>

You can launch it in debug mode and normal mode </br>
In debug mode gazebo model state is used to populate sensor messages </br>
so that slam can be tested without worrying about data association </br>

to launch only landmark detector::</br>
<pre><code>roslaunch nuslam landmarks.launch robot:=-1 
</code></pre>
It looks as follows </br>

![image](nuslam/images/landmark_detector.png)


Robot argument is dependent on choice of platform </br>
this launch file also starts a keyboard teleop system which can be used to </br>
navigate the bot</br>


to launch slam 

<pre><code>roslaunch nuslam slam.launch robot:=-1 debug:=True </pre></code>

debug argument can be changed as needed