# Extroperceptive_sensor_calibration
This is a package for calibrating multiple extroperceptive sensors

This code is originally for mulitple 2D LIDAR calibration.  
Now this package is refined and expanded into multiple sensor case such as (LIDAR-LIDAR,  LIDAR to RGBD, 2DLIDAR to 3DLIDAR and LIDAR to stereo)

So far it uploaded with 3 sensor case. To use more, feel free to add more planner constraint cases in the optimization function 

Only tested with Ubuntu 16.04, MRPT 1.5 and ROS-Kietic


Do not use sudo apt get way to get library as there is multiple machine depended case which prevents you from compiling successfully.
Do not download mrpt_bridge as it still has bugs in converting timestamp.
Build MRPT 1.5 on your own PC. 

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

catkin_make


To run 

cd catkin_ws

source ./devel/setup.bash

rosrun lidartocameracalib lidartocameracalib sensor.ini rawlog


the code was partially adopted from 

Eduardo Fernández-Moral, Javier González-Jiménez, Vicente Arévalo. Extrinsic calibration of 2D laser rangefinders from perpendicular plane observations. Int. J. Rob. Res. 34, 11 (September 2015), 1401-1417. DOI: https://doi.org/10.1177/0278364915580683

Their original code is broken and was not being repair for long time. 






