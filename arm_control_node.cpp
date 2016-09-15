//This Node is for controlling the arm of fetch depending on the information from the Target Object Coordinates Topic and the Ultrasonic Distance Topic

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"

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
void coordinatesCallBack();
void distanceCallBack();
void ROS_Publisher();

using namespace std;


//>>>function to calculate the movement of the arm's servos depending on the coordinates and the ultrasonic distance
int calculate_movement(){

}
//<<<function to calculate the movement of the arm's servos depending on the coordinates and the ultrasonic distance

//>>>function to send the calculated movement to the arduino that controls the arm's servos
void send_movement(int arm_position_data){
  int data[] = {arm_position_data} //data that we want to send
  FILE *file;
  file = fopen("/dev/ttyACM0", "w"); //opens the serial port stream to the arduino Mega
  if(file == NULL){
    cout<<"ERROR: cannot connect to /dev/ttyACM0"<<endl;
    exit(EXIT_FAILURE);
  }

  int i = 0;
  for(; i<4; i++){
    fprintf(file, "%d", data[i]); //write to the arduino
    fprintf(file, "%c", ','); //separate the digits
    cout<<"data "<<i<<" sent: "<<data[i]<<endl;
    sleep(1); //sleep so that the data can properly be sent without hold up
  }
  fclose(file);
}
//<<<function to send the calculated movement to the arduino that controls the arm's servos

//>>>ROS callback function for the target object coordinates
void coordinatesCallBack(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard coordinates: [%s]", msg->data.c_str());
}
//<<<ROS callback function for the target object coordinates

//>>>ROS callback function for ultrasonic distance
void distanceCallBack(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard distance: [%s]", msg->data.c_str());
}
//<<<ROS callback function for ultrasonic distance

//>>>ROS subscriber code
void ROS_Subscriber(){
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
  ros::subscriber sub = n.subscribe("target_object_coordinates", 1000, coordinatesCallBack);
  ros::subscriber sub = n.subscribe("", 1000, distanceCallBack);

  //this will enter a loop, pumping callbacks
  ros::spin(); //only exits with Ctrl-C or if the node is shutdown by the master


}
//<<<ROS subscriber code

//>>> ROS publisher code, this is for telling when the object has been obtained
//TODO: figure out if I really need this -Roy Xing September 3, 2016
//<<< ROS publisher code

//>>> int main
int main(int argc, char **argv){

}
//<<< int main
