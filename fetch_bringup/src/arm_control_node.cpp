//This Node is for controlling the arm of fetch depending on the information from the Target Object Coordinates Topic and the Ultrasonic Distance Topic

//Author: Roy Xing

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"

//C++ stuff
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <math.h>

//OpenCV stuff
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>

#define PI 3.14159265

using namespace std;

//initiate the functions
int calculate_movement_x(int x_coordinate);
int calculate_movement_y(int y_coordinate);
void calculate_movement(int x_coor, int y_coor, float ultrasonic);
void ROS_Subscriber();
void ROS_Publisher();
void coordinatesCallBack(const std_msgs::Int32MultiArray::ConstPtr& coordinates);
void distanceCallBack(const std_msgs::Float32::ConstPtr& distance);
void arm_movement(int argc, char **argv);


//global variables
bool targetClose = false;
int posClaw = 90;
int posElbow = 90;
int posShoulder = 90;
int posBase = 90;
int x_coordinate = 0;
int y_coordinate = 0;
float ultra_distance = 0;


//>>>function to calculate and set the positions of the arm to send to the arm
void calculate_movement(int x_coor, int y_coor, float ultrasonic){
  //NOTE: every time the shoulder moves by ___ the elbow moves by ___
  if(int(ultrasonic) < 21){ //assumes that ultrasonic is measured in cm
    ultrasonic = ultrasonic - 12;
    posElbow = asin(ultrasonic / 16) * 180.0 / PI + 45;
    posShoulder = 180;
    posClaw = 120;
  }
}
//<<<function to calculate and set the positions of the arm to send to the arm


//>>>function to calculate the movement of the arm's servos depending on the x coordinate and the ultrasonic distance
int calculate_movement_x(int x_coordinate, int USDistance){


  /*
  int arm_position_data;
  if(x_coordinate > 0 && x_coordinate < 250){ //if the target object is to the left of fetch's arm camera
    arm_position_data = 135;
  }
  else if(x_coordinate > 450){ //if the target object is in front of fetch's arm camera
    arm_position_data = 45;
  }
  else if(x_coordinate < 450 && x_coordinate > 250){ //if the target object is to the right of fetch's arm camera
    arm_position_data = 90;
  }
  //int arm_position_data = 45;
  return arm_position_data;
  */
}
//<<<function to calculate the movement of the arm's servos depending on the x coordinate and the ultrasonic distance


//>>>function to calculate the movement of the arm's servos depending on the y coordinate and the ultrasonic distance
int calculate_movement_y(int y_coordinate){
  int arm_position_data = y_coordinate;
  return arm_position_data;
}
//<<<function to calculate the movement of the arm's servos depending on the y coordinate and the ultrasonic distance


//>>>function to send the calculated movement to the arduino that controls the arm's servos
void send_movement(int posBase, int posShoulder, int posElbow, int posClaw){
  FILE *file;
  int data[] = {posBase, posShoulder, posElbow, posClaw};
  file = fopen("/dev/ttyACM0", "w"); //opens the serial port stream to the arduino Mega
  if(file == NULL){
    cout<<"ERROR: cannot connect to /dev/ttyACM0"<<endl;
    exit(EXIT_FAILURE);
  }

  for(int i = 0; i < 4; i++){
    fprintf(file, "%d", data[i]); //write to the arduino
    fprintf(file, "%c", ','); //separate the digits
    cout<<"sent: "<< i << ", " << data[i] <<endl;
  }

  sleep(1); //sleep so that the data can properly be sent without hold up

  fclose(file);
}
//<<<function to send the calculated movement to the arduino that controls the arm's servos


//>>>ROS callback function for the target object coordinates
void coordinatesCallBack(const std_msgs::Int32MultiArray::ConstPtr& coordinates){
  x_coordinate = coordinates->data[0];
  y_coordinate = coordinates->data[1];
}
//<<<ROS callback function for the target object coordinates


//>>>ROS callback function for ultrasonic distance
void distanceCallBack(const std_msgs::Float32::ConstPtr& distance){
  //ROS_INFO("I heard distance: [%d]", distance->data.c_str());
  ROS_INFO("I heard distance: %f", distance->data);

  ultra_distance = distance->data;

  cout<<"distance: "<<ultra_distance<<endl;
}
//<<<ROS callback function for ultrasonic distance


//>>>ROS publisher arm position data to Serial_Communication Python node
void ROS_Publisher(int argc, char **argv){
  ros::init(argc, argv, "arm_control_node");
  ros::NodeHandle n;
  ros::Publisher pub_posData = n.advertise<std_msgs::Int32MultiArray>("posData", 100);
  while(ros::ok()){
    std_msgs::Int32MultiArray posData;
    posData.data.clear();
    posData.data.push_back(posBase);
    posData.data.push_back(posShoulder);
    posData.data.push_back(posElbow);
    posData.data.push_back(posClaw);
    pub_posData.publish(posData);
    ROS_INFO("Published posData");
    ros::spinOnce();
    sleep(2);
  }
}
//<<<ROS publisher arm position data to Serial_Communication Python node


//>>> Main Program
void arm_movement(int argc, char **argv){
  //initiate a node called "Arm_Control_Node"
  ros::init(argc, argv, "Arm_Control_Node");
  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  //subscribers
  ros::Subscriber sub_coordinates = n.subscribe("coordinates", 10, &coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("ultrasonic_distance", 10, &distanceCallBack);

  //publishers
  ros::Publisher pub_posData = n.advertise<std_msgs::Int32MultiArray>("posData", 4);

  ros::Rate loop_rate(10);

  //array with the arm positions to be sent to the arduino
  std_msgs::Int32MultiArray posData;

  while(ros::ok()){

    if((x_coordinate < 480 && x_coordinate > 160) && (y_coordinate > 270)){
      cout<<"I see the target object"<<endl;
      calculate_movement(x_coordinate, y_coordinate, ultra_distance);
      targetClose = true;
    }

    ROS_INFO("Heard Camera Coordinates: X: %d, Y: %d", x_coordinate, y_coordinate);

    if(targetClose == true){
      posData.data.clear();

      for(int i = 0; i < 4; i++){
        //posData.data.push_back(rand() % 180);
        switch(i){
        case 1:
          posData.data.push_back(posBase);
          break;
        case 2:
          posData.data.push_back(posShoulder);
          break;
        case 3:
          posData.data.push_back(posElbow);
          break;
        case 4:
          posData.data.push_back(posClaw);
          break;
        }
      }
      pub_posData.publish(posData);
      ROS_INFO("Published posData");
      cout<<posData<<endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
//<<< Main Program


//>>> int main
int main(int argc, char **argv){
  //initiate a node called "Arm_Control_Node"
  ros::init(argc, argv, "Arm_Control_Node");
  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  //subscribers
  ros::Subscriber sub_coordinates = n.subscribe("objectCoordinates", 10, &coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("ultrasonic_distance", 10, &distanceCallBack);

  //publishers
  ros::Publisher pub_posData = n.advertise<std_msgs::Int32MultiArray>("posData", 4);

  ros::Rate loop_rate(10);

  //array with the arm positions to be sent to the arduino
  std_msgs::Int32MultiArray posData;

  while(ros::ok()){
    if((x_coordinate < 480 && x_coordinate > 160) && (y_coordinate > 270) && ultra_distance < 18){
      ROS_INFO("I see the target Object at (%d, %d)", x_coordinate, y_coordinate);
      calculate_movement(x_coordinate, y_coordinate, ultra_distance);
      targetClose = true;
    }
    else{
      ROS_INFO("Heard Camera Coordinates: X: %d, Y: %d", x_coordinate, y_coordinate);
    }
    if(targetClose == true){
      posData.data.resize(4);
      posData.data.clear();
      for(int i = 0; i < 4; i++){
        //posData.data.push_back(rand() % 180);
        switch(i){
        case 1:
          posData.data.push_back(posBase);
          break;
        case 2:
          posData.data.push_back(posShoulder);
          break;
        case 3:
          posData.data.push_back(posElbow);
          break;
        case 4:
          posData.data.push_back(posClaw);
          break;
        }
      }
      pub_posData.publish(posData);
      ROS_INFO("Published posData");
      cout<<posData<<endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
//<<< int main
