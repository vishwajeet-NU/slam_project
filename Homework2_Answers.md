"ME495 Sensing, Navigation, and Machine Learning"
Author : Vishwajeet Karmarkar </br>
# Answers 
# F.007, 3
# Data: 

Name		FV	AV	ET	EX	EY	OT		OX	OY	FT	FX	FY	GT	GX	GY	DT		DX		DY
Vishwajeet-FWD	1	0.22	0	2	0	0		1.984	0	0	1.976	0	0	1.85	0	0		-0.0134		0
Vishwajeet-FWD	0.6	0.132	0	2	0	0		2.103	0	0	1.995	0	0	2.08	0	0		-0.0023		0
Vishwajeet-RWD	1	0.22	0	-2	0	0		-1.894	0	0	-1.984	0	0	-1.9	0	0		-0.0006		0
Vishwajeet-RWD	0.6	0.132	0	-2	0	0		-2.115	0	0	-1.989	0	0	-2.11	0	0		0.0005		0
Vishwajeet-CCW	1	2.84	0	0	0	177.619		0	0	-7.776	0	0	135	0	0	-2.13095	0		0
Vishwajeet-CCW	0.6	1.704	0	0	0	42.214		0	0	-14.284	0	0	30	0	0	-0.6107		0		0
Vishwajeet-CW	1	2.84	0	0	0	-158.418	0	0	6.148	0	0	-164	0	0	-0.2791		0		0
Vishwajeet-CW	0.6	1.704	0	0	0	-45.365		0	0	16.237	0	0	-35	0	0	0.51825		0		0'

# F.007, 6
Yes Including encoders does improve the pose estimates drastically. This was seen visually on comparing fake_odom from fake_diff_econders 
and odom while the turtle was moving, on ground and in rviz. The ground reality is captured quite closely by the encoders which fed into
odom. An example from data, is rotation ccw, speed 1, where fake encoders expect a position of -7.776 deg, while the real orientation is
close to 135 deg, which is estimated by the encoders to be 177.619, which is much more accurate.

# F.007, 7
Yes, slowing speed does improve the odometry. This is seen very clearly from the data above, eg ccw 1 with Dt of -2.13 for full speed, compared 
to dt of -0.61 for 0.6 times the full speed, in the same direction of rotation.


#F.009 

rviz : https://www.youtube.com/watch?v=zViuKk1AgLk
real : https://www.youtube.com/watch?v=zViuKk1AgLk

