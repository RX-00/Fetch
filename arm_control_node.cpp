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
#include <vector>
#include <algorithm>
#include <math.h>

//OpenCV stuff
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>

//initiate the functions
void send_movement();
int calculate_movement();
void ROS_Subscriber();
void coordinatesCallBack(const std_msgs::Int32MultiArray::ConstPtr& coordinates);
void distanceCallBack();
void ROS_Publisher();
void arm_movement();

using namespace std;

//Global Variable
int servo_position;


//>>>function to calculate the movement of the arm's servos depending on the coordinates and the ultrasonic distance
int calculate_movement(int x_coordinate){
  int arm_position_data = x_coordinate;
  return arm_position_data;
}
//<<<function to calculate the movement of the arm's servos depending on the coordinates and the ultrasonic distance

//>>>function to send the calculated movement to the arduino that controls the arm's servos
void send_movement(int arm_position_data){
  int data[] = {arm_position_data}; //data that we want to send
  FILE *file;
  file = fopen("/dev/ttyACM0", "w"); //opens the serial port stream to the arduino Mega
  if(file == NULL){
    cout<<"ERROR: cannot connect to /dev/ttyACM0"<<endl;
    exit(EXIT_FAILURE);
  }

  int i = 0;
  for(; i<1; i++){
    fprintf(file, "%d", data[i]); //write to the arduino
    fprintf(file, "%c", ','); //separate the digits
    cout<<"data "<<i<<" sent: "<<data[i]<<endl;
    sleep(1); //sleep so that the data can properly be sent without hold up
  }
  fclose(file);
}
//<<<function to send the calculated movement to the arduino that controls the arm's servos

//>>>ROS callback function for the target object coordinates
void coordinatesCallBack(const std_msgs::Int32MultiArray::ConstPtr& coordinates){
  //ROS_INFO("I heard coordinates: [%d]", x_coordinate->data.c_str());
  ROS_INFO("I heard coordinates: x coordinate: %d", coordinates->data[0]);
  ROS_INFO("                     y coordinate: %d", coordinates->data[1]);

  //cout<<x_coordinate->data<<endl;

  //algorithm for tracking the target object
  //NOTE: Keep the target object in sight (center to the arm), until fetch is facing the target object and the arm base servo is around 90 degrees

  //Y Coordinate based movement of the arm

  //X Coordinate based movement of the arm
  if(coordinates->data[0] > 100 && coordinates->data[0] < 200){ //if the target object is in front of fetch's arm camera
    servo_position = calculate_movement(coordinates->data[0]);
    send_movement(servo_position);
  }
  else if(coordinates->data[0] > 200){ //if the target object is to the right of fetch's arm camera
    servo_position = calculate_movement(coordinates->data[0]);
    send_movement(servo_position);
  }
  else if(coordinates->data[0] < 100){ //if the target object is to the left of fetch's arm camera
    servo_position = calculate_movement(coordinates->data[0]);
    send_movement(servo_position);
  }

}
//<<<ROS callback function for the target object coordinates

//>>>ROS callback function for ultrasonic distance
void distanceCallBack(const std_msgs::Int32::ConstPtr& distance){
  //ROS_INFO("I heard distance: [%d]", distance->data.c_str());
  ROS_INFO("I heard distance: %d", distance->data);

  //algorithm for when to pick up the target object
  if((servo_position < 95 && servo_position > 85) && distance->data < 100){
    int go_grab_object = -1;
    send_movement(go_grab_object);
  }
}
//<<<ROS callback function for ultrasonic distance

//>>>ROS subscriber code, NOTE: the beginning of this should be in the beginning of the "main program"
void ROS_Subscriber(int argc, char **argv){
  //initiate a node called "Arm_Control_Node"
  ros::init(argc, argv, "Arm_Control_Node");

  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  /*The subscribe() call is how you tell ROS you want to receive messages on a given topic
  -Invokes a call to the ROS master node
  -Messages are passed to callback function
  -subscribe() returns a Subscriber object that you must hold on to until you want to unsubscribe
  - When all copies of the Subscribed obbject go out of scope, this callback will automatically be unsubscribed form this topic
  -the second parameter to the subscribe() function is the size of the message queue
  */
  ros::Subscriber sub_coordinates = n.subscribe("target_object_coordinates", 1000, coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("target_object_distance", 1000, distanceCallBack);

  //this will enter a loop, pumping callbacks
  ros::spin(); //only exits with Ctrl-C or if the node is shutdown by the master
}
//<<<ROS subscriber code

//>>> ROS publisher code, this is for telling when the object has been obtained
//TODO: figure out if I really need this -Roy Xing September 3, 2016
//<<< ROS publisher code



//>>> Main Program
void arm_movement(int argc, char **argv){
  //initiate a node called "Arm_Control_Node"
  ros::init(argc, argv, "Arm_Control_Node");
  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  //subscribers
  ros::Subscriber sub_coordinates = n.subscribe("target_object_coordinates", 1000, coordinatesCallBack);
  ros::Subscriber sub_distance = n.subscribe("", 100, distanceCallBack);
  ros::spin();
}
//<<< Main Program


//>>> int main
int main(int argc, char **argv){
  arm_movement(argc, argv);
  return 0;
}
//<<< int main
