# Extroperceptive_sensor_calibration
This is a package for calibrating multiple extroperceptive sensors
![alt tag](https://github.com/snakehaihai/Extroperceptive_sensor_calibration/blob/master/output/process.png) 
![alt tag](https://github.com/snakehaihai/Extroperceptive_sensor_calibration/blob/master/output/result.png) 

This code was deisgned for mulitple 2D lidar calibration and now it is expanded for general extroperceptive sensors cases such as 
(2DLIDAR-2DLIDAR,  2DLIDAR to RGBD, 2DLIDAR to 3DLIDAR and 2DLIDAR to stereo).

The uploaded version is build with 3 sensor case. To use more, feel free to add more 
Planner constraint and Constraint Jacobian in the optimization function.  


# Only tested with Ubuntu 16.04, MRPT 1.5 and ROS-Kietic

Do not use sudo apt get way to get library as MRPT is heavy compiler depended.
Do not download mrpt_bridge as it still has bugs in converting timestamp.
Build MRPT 1.5 on your own PC. (Frankly speaking MRPT is one of the most buggy platform with better math integration. If you have MRPT related problem. Please open an issue there. Usually takes 2 to 6 month b4 any 1 will reply you) 

git clone https://github.com/MRPT/mrpt/tree/mrpt-1.5

sudo apt install build-essential pkg-config cmake libwxgtk3.0-dev \
libopencv-dev libeigen3-dev libgtest-dev
sudo apt install libftdi-dev freeglut3-dev zlib1g-dev libusb-1.0-0-dev \
libudev-dev libfreenect-dev libdc1394-22-dev libavformat-dev libswscale-dev \
libassimp-dev libjpeg-dev   libsuitesparse-dev libpcap-dev liboctomap-dev

then 

mkdir build && cd build
cmake ..
make



Download two of the files in ros catkin_ws.
then. 
modify the directory in the CMAKELIST into your own build directory. If you only use one version of MRPT and it is make install into your OS, remove the path link in the CMAKELIST.


catkin_make


# To run 

cd catkin_ws

source ./devel/setup.bash

rosrun lidartocameracalib lidartocameracalib sensor.ini rawlog

The sample given is 3LIDAR case
To use it with your own data
1. Record your data into the rawlog. Remember to add sensor frame ID and many other things such as FOV, aperture size etc

2. Write a sensor init file specifiy the inital guess of each sensor

3. call it with     rosrun  lidartocameracalib lidartocameracalib your_sensor.ini your_sensor_rawlog


# Failure case
1. If your initial guess is too far from the actual case, there is a high chance it might not find a proper result. Try to input a proper one
2. If somehow the resulting fisher information matrix goes into the degenerative case or put the other word, det(FIM)=0 . There would be no solution as the hessian is non-invertible. Thus the update process can not run.  Try to record another set of data
3. Sensor data reqires to have a at least 1 cross point for each of the two sensor. For total none overlap case, there is no way to calibrate them without ground truth position. 

# Disclaimer
We do not grauntee 100% working of the code. Feel free to modifiy and use. But we will not be taken responsible for any drone crash, car crash or any damage caused by this package. Use it at your own risk!!!



The code was partially adopted from 

Eduardo Fernández-Moral, Javier González-Jiménez, Vicente Arévalo. Extrinsic calibration of 2D laser rangefinders from perpendicular plane observations. Int. J. Rob. Res. 34, 11 (September 2015), 1401-1417. DOI: https://doi.org/10.1177/0278364915580683

Their original code is broken and was not fixed for a long time. Feel free to take a look at their orignal code



