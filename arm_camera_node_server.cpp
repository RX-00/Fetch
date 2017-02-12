//This Node is for the camera on top of the arm of Fetch
//The camera locates where the target object to fetch is and its coordinates
//The coordinates are then sent to a client, arm_control_node_client to be sent to an the robotic arm of Fetch

//Author: Roy Xing

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

//C++ stuff
#include <iostream>
#include <sstream>
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

using namespace cv;
using namespace std;
using namespace ros;


//Global Variables for the color detection initiation defaults
int LowH = 40;
int HighH = 80;

int LowS = 0;
int HighS = 90;

int LowV = 100;
int HighV = 255;

//Global Variables for the default values of the previous cooridnate of the target object
int LastX = -1;
int LastY = -1;


//initiate your functions
//void find_target_object(int argc, char **argv);
//void create_trackbars();

/*
//>>>trackbars for filtering bsaed on color
void create_trackbars(){
  //create window named Control Colors
  cv::namedWindow("Control Colors", CV_WINDOW_AUTOSIZE);
  //Create the trackbars
  //cvCreateTrackbar("LowH", "Control Colors", &LowH, 179);
  //cvCreateTrackbar("HighH", "Control Colors", &HighH, 179);

  //cvCreateTrackbar("LowS", "Control Colors", &LowS, 255);
  //cvCreateTrackbar("HighS", "Control Colors", &HighS, 255);

  //cvCreateTrackbar("LowV", "Control Colors", &LowV, 255);
  //cvCreateTrackbar("HighV", "Control Colors", &HighV, 255);
}
//<<<trackbars for filtering based on color

/*
//>>>image processing from the camera
void find_target_object(int argc, char **argv){
  ros::init(argc, argv, "arm_camera_node_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_camera_node_server", add);
  ROS_INFO("Ready to send camera & object info info");

}
//<<<image processing from the camera

//>>>service for taking in the request from the arm_control_node_client and response type defined in the srv file and returns a boolean
bool camera(fetch::arm_camera_node_server::Request &req, fetch::arm_control_node_client::Response &res){

  return true;
}
//<<<service for taking in the request from the arm_control_node_client and response type defined in the srv file and returns a boolean
*/

//>>>int main
int main(int argc, char **argv){
  //find_target_object(argc, argv);
  return 0;
}
//<<<int main

