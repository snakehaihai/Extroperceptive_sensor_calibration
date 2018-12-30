# Extroperceptive_sensor_calibration
This is a package for calibrate multiple extroperceptive sensors

Only tested with Ubuntu 16.04, MRPT 1.5 and ROS-Kietic


Do not use sudo apt get way to get library as there are multiple machine depended case which prevent you from compile succesfull.
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





