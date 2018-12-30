# Extroperceptive_sensor_calibration
This is a package for calibrating multiple extroperceptive sensors

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

Eduardo Fern$#225;ndez-Moral, Javier Gonz$#225;lez-Jim$#233;nez, and Vicente Ar$#233;valo. 2015. Extrinsic calibration of 2D laser rangefinders from perpendicular plane observations. Int. J. Rob. Res. 34, 11 (September 2015), 1401-1417. DOI: https://doi.org/10.1177/0278364915580683

Their original code was not able to compile. In this package, the code is refined and can be compiled and run in Ubuntu 16.04. Multiple types of the sensor is allowed to calibrate at the same time. 
So far it uploaded with 3 sensor case. To use more, feel free to add more planner constraint cases in the optimization function 





