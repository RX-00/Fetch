# Fetch
An Autonomous Robot That Fetches Objects




INFO
The way I installed kinect for ROS
https://bi.snu.ac.kr/Courses/4ai12s/Projects/KinectWithROSTutorial.pdf
http://wiki.ros.org/openni_launch
http://wiki.ros.org/openni_kinect
Maybe http://wiki.ros.org/openni_camera
https://richardstechnotes.wordpress.com/2015/11/22/using-the-kinect-and-ros-openni_camera-on-the-raspberry-pi-2/

Uses ROS Indigo


C++ Compiler, g++
g++ `pkg-config opencv --cflags` shaky_camera_HSV_BGR.cpp -o shaky_camera_HSV_BGR.out `pkg-config opencv --libs` -I/usr/local/include/opencvblobslib -L/usr/local/lib -lopencvblobslib -std=c++11

Gmapping demo commands
roscore
roslaunch chefbot_gazebo chefbot_hotel_world.launch
roslaunch chefbot_gazebo gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
