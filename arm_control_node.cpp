//This Node is for controlling the arm of fetch depending on the information from the Target Object Coordinates Topic and the Ultrasonic Distance Topic

//Author: Roy Xing

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

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

//DEFINE VARIABLES
#define BASE 98 //send to the arduino to use the arm base servo
#define SHOULDER 97 //send to the arduino to use the arm shoulder servo
#define ELBOW 96 //send to the arduino to use the arm elbow servo
#define ALL 95 //send to the arduino to use all servos
#define ARM 94 //send to the arduino to use the arm elbow and shoulder servos

using namespace std;

//initiate the functions
int calculate_movement_x(int x_coordinate);
int calculate_movement_y(int y_coordinate);
void calculate_movement(int x, int y, int ultrasonic);
void ROS_Subscriber();
void ROS_Publisher();
void coordinatesCallBack(const std_msgs::Int32MultiArray::ConstPtr& coordinates);
void distanceCallBack(const std_msgs::Int32::ConstPtr& distance);
void ROS_Publisher();
void arm_movement(int argc, char **argv);


//global variables
int posClaw;
int posElbow;
int posShoulder;
int posBase;
int x_coordinate;
int y_coordinate;


//>>>function to calculate and set the positions of the arm to send to the arm
void calculate_movement(int x, int y, int ultrasonic){
  int calcElbow, calcShoulder;



  posClaw = 120;
  posElbow = calcElbow;
  posShoulder = calcShoulder;
  posBase = 90;
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
  int servo_position;
  //ROS_INFO("I heard coordinates: [%d]", x_coordinate->data.c_str());
  ROS_INFO("I heard coordinates: x coordinate: %d", coordinates->data[0]);
  ROS_INFO("                     y coordinate: %d", coordinates->data[1]);

  x_coordinate = coordinates->data[0];
  y_coordinate = coordinates->data[1];

  //cout<<x_coordinate->data<<endl;

  //algorithm for tracking the target object
  //NOTE: Keep the target object in sight (center to the arm), until fetch is facing the target object and the arm base servo is around 90 degrees

  //Y Coordinate based movement of the arm
  //NOTE: 250 is the middle of the y axis, min of 10, max of 460 (for the program)

  if(coordinates->data[1] > 400){
    cout<<"target object is close"<<endl;
    //servo_position = calculate_movement_y(coordinates->data[0]);
  }


  //X Coordinate based movement of the arm
  //NOTE: 350 is the middle of the x axis, min of 10, max of 600 (for the program)

  //servo_position = calculate_movement_x(coordinates->data[0]);

}
//<<<ROS callback function for the target object coordinates


//>>>ROS callback function for ultrasonic distance
void distanceCallBack(const std_msgs::Int32::ConstPtr& distance){
  //ROS_INFO("I heard distance: [%d]", distance->data.c_str());
  ROS_INFO("I heard distance: %d", distance->data);

  int ultra_distance = distance->data;

  //algorithm for when to pick up the target object
  if(ultra_distance < 100 && y_coordinate > 250){
    int go_grab_object = -1;
    //send_movement(ALL, go_grab_object);
  }
}
//<<<ROS callback function for ultrasonic distance


//>>>ROS subscriber code, NOTE: the beginning of this should be in the beginning of the "main program"
void ROS_Subscriber(int argc, char **argv){
  //initiate a node called "Arm_Control_Node"
  ros::init(argc, argv, "arm_control_node");

  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  /*The subscribe() call is how you tell ROS you want to receive messages on a given topic
  -Invokes a call to the ROS master node
  -Messages are passed to callback function
  -subscribe() returns a Subscriber object that you must hold on to until you want to unsubscribe
  - When all copies of the Subscribed obbject go out of scope, this callback will automatically be unsubscribed form this topic
  -the second parameter to the subscribe() function is the size of the message queue
  */
  ros::Subscriber sub_coordinates = n.subscribe("target_object_coordinates", 10, coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("target_object_distance", 10, distanceCallBack);

  //this will enter a loop, pumping callbacks
  ros::spin(); //only exits with Ctrl-C or if the node is shutdown by the master
}
//<<<ROS subscriber code


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
  ros::init(argc, argv, "arm_control_node");
  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  /*
  //subscribers
  ros::Subscriber sub_coordinates = n.subscribe("target_object_coordinates", 1000, coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("", 100, distanceCallBack);
  ros::spin();
  */

  //publishers
  ros::Publisher pub_posData = n.advertise<std_msgs::Int32MultiArray>("posData", 4);
  while(ros::ok()){
    std_msgs::Int32MultiArray posData;
    posData.data.clear();

    /*
    posData.data.push_back(posBase);
    posData.data.push_back(posShoulder);
    posData.data.push_back(posElbow);
    posData.data.push_back(posClaw);
    */

    for(int i = 0; i < 4; i++){
      posData.data.push_back(rand() % 180);
    }

    pub_posData.publish(posData);
    ROS_INFO("Published posData");
    cout<<posData<<endl;
    ros::spinOnce();
    sleep(10);
  }
}
//<<< Main Program


//>>> int main
int main(int argc, char **argv){
  arm_movement(argc, argv);

  /*
  while(true){
    send_movement(BASE, 135);
    sleep(5);
    send_movement(BASE, 45);
    sleep(5);
  }
  */

  return 0;
}
//<<< int main
