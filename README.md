# Disclaimer
We do not grauntee 100% working of the code. Feel free to modifiy and use. But we will not be taken responsible for any drone crash, car crash, life loss or any damage caused by this package. Use and modify it at your own risk!!!

# Extroperceptive_sensor_calibration
This is a package for calibrating multiple extroperceptive sensors
![alt tag](https://github.com/snakehaihai/Extroperceptive_sensor_calibration/blob/master/output/process.png) 
![alt tag](https://github.com/snakehaihai/Extroperceptive_sensor_calibration/blob/master/output/result.png) 

This code was deisgned for mulitple 2D lidar calibration and now it is expanded for general extroperceptive sensors cases such as 
(2DLIDAR-2DLIDAR,  2DLIDAR to RGBD, 2DLIDAR to 3DLIDAR and 2DLIDAR to stereo).

The uploaded version is build with 3 sensor case. To use more, feel free to add more 
Planner constraint and Constraint Jacobian in the optimization function. Each new sensor need to add at least 1 planner constaint function

# Build
 Only tested with Ubuntu 16.04, MRPT 1.5 and ROS-Kietic

Do not use sudo apt get way to get library as MRPT is heavy gcc-version depended.
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


# Run 

cd catkin_ws

source ./devel/setup.bash

rosrun lidartocameracalib lidartocameracalib sensor.ini rawlog

# To use with custom sensor sets
The sample given is 3LIDAR case. For 2 x 2D LIDAR and one RGBD camera case is given in the other directory called lidartocameracalib_recorddata, you can take a look at how to process it. 
To use it with your own data
1. Record your data into the rawlog. Remember to add sensor frame ID and many other things such as FOV, aperture size etc

2. Write a sensor init file specifiy the inital guess of each sensor

3. call it with     rosrun  lidartocameracalib lidartocameracalib your_sensor.ini your_sensor_rawlog


# Failure case
1. If your initial guess is too far from the actual case, there is a high chance it might not find a proper result. Try to input a proper one
2. If somehow the resulting fisher information matrix goes into the degenerative case or put the other word, det(FIM)=0 . There would be no solution as the hessian is non-invertible. This can happen quite often when all sensor reports the same right angle at same time. Thus the update process can not run.  Try to record another set of data with slightly modified position
3. Sensor data reqires to have a at least 1 cross point for each of the two sensor. For total none overlap case, there is no way to calibrate them. 





