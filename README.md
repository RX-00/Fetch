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


For use of MPU6050 IMU
https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
https://www.i2cdevlib.com/devices/mpu6050


Blog:
http://royxingrobots.blogspot.com/


Hardware:

-Base, based on Qbotics' chefbot, turtlebot

-Xbox 360 Kinect sensor

-NTE966 12V 1A Voltage Regulator + 0.33μF and 0.1μF capacitors (for Xbox 360 Kinect sensor http://wiki.ros.org/kinect/Tutorials/Adding%20a%20Kinect%20to%20an%20iRobot%20Create)

-Tiva C Launchpad TM4C123G (for sensors and motors/motordriver)

-2x 131:1 Metal Gearmotor 37Dx73L mm with 64 CPR Encoder

-2x Pololu Stamped Aluminum L-Bracket Pair for 37D mm Metal Gearmotors

-Dual VNH2SP30 Motor Driver Carrier MD03A

-4x Logic Level Converter - Bi-Directional 3.3V 5V

-RioRand LM2596 DC-DC Buck Converter Step Down Module Power Supply Output 1.23V-30V

-GY-521 MPU-6050 3 Axis gyroscope

-Ultrasonic Module HC-SR04

-Turnigy 5000mAh 4S1P 14.8v 20C Hardcase Pack

-Arduino MEGA 2560

-Arduino Servo and Sensor Shield v5.0

-MeArm Open Source Robotic Arm (laser cut wood) w/ 4x TowerPro SG90 9G Micro Servo

-Disassembled Frontech JIL- 2244 Webcam, (salvaged the camera module board w/o LEDs)

-standard electronics: wires (jumper, USB A to USB B, alligator (for battery to breadboard), etc.), breadboard, heatshrink, etc.

