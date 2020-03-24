Vishwajeet Karmarkar
Email: vishwajeetkarmarkar2021@u.northwestern.edu

This package builds a simple differential drive robot model, that can be visualized, and simulated 
in Rviz. It contains a xacro which loads parameters from a yaml file, making the diff-bot dimensions 
alterable, without having to touch the file. This is potentially useful, as the yaml generation can 
be automated, thus creating modularity. 

Command: 
use the following command to run the package and see the robot in Rviz

roslaunch nuturtle_description view_diff_drive.launch


files: 
1) config files
	a) diff_params.yaml : contains dimensions that help build the diff_drive robot. Modfiy these to 
           change the robot dimensions
	b) urdf.rviz: these contains the saved configuration of rviz. Save from the Rviz gui any changes
	   if required, and those will be loaded up for the next time
2) launch files:
	a) view_diff_drive.launch: this file launches the nodes joint_state_publisher and robot_state_publisher
	   along with Rviz, in one go. It also loads paramters stated in the yaml file into the parameter 
	   server
3) urdf: 
	a) diff_drive.urdf.xacro : this file contains the robot model. It is paramteric as it can take in values
	   from the yaml file, and change the robot model accordingly 

