### "ME495 Sensing, Navigation, and Machine Learning" </br> ###

Author: Vishwajeet Karmarkar </br>
vishwajeetkarmarkar2021@u.northwestern.edu: </br>

Results: </br>
L.002:</br>
Detection radius = 2
sensor co_variance = 0.000001 </br>
gaussian noise variance laser scan = 0.01 </br>
motion model noise = 0.0000001 </br>

Error 1, Actual - Slam : </br>
x_error = -0.0528m </br>
y_error = -0.0379m </br>

Error, Slam - Odom : </br>
x_error = 0.00 </br>
y_error = 0.005 </br>

Error 2, Actual - Odom : </br>
x_error = -0.0528m </br>
y_error = -0.0373m </br>


![image](nuslam/images/with_error.png)

</br>

Another run with lower noise and slower movement 
![image](nuslam/images/3.png)

Green : slam path </br>
Brown: ground path </br>
Blue : odom path </br>

Markers: </br>
blue : landmarks seen by sensor </br>
red  : landmarks in slams, state vector </br>