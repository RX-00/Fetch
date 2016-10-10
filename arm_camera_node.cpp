//This Node is for the camera on top of the arm of Fetch
//The camera locates where the target object to fetch is and its coordinates
//The coordinates are then published to a topic to be used by other nodes

//Author: Roy Xing

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

//C++ stuff
#include <iostream>
#include <sstream>
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

/*------------------------
  Yes, I know I shouldn't
  rely on these too much,
  I'll try my best not to
  depend on these

  Gotta keep up that
  good coding practice
  ya'know

  -Roy Xing
  August 23, 2016
  ------------------------*/
using namespace cv;
using namespace std;
using namespace ros;


//Global Variables for the color detection initiation defaults
int LowH = 0;
int HighH = 179;

int LowS = 45;
int HighS = 255;

int LowV = 110;
int HighV = 255;

//Global Variables for the default values of the previous cooridnate of the target object
int LastX = -1;
int LastY = -1;



//initiate your functions
float distance_formula();
//int find_target_object();
void find_target_object();
void createTrackbars();
void ROS_Publisher();

//>>>Euclidian Distance Formula
float distance_formula(float x1, float x2, float y1, float y2){
  float distance;
  distance = sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
  return distance;
}
//<<<Euclidian Distance Formula


//>>>trackbars for filtering based on color
void createTrackbars(){
  //create window
  namedWindow("Control Colors", CV_WINDOW_AUTOSIZE);
  //I just want to forget
  //Create the trackbars
  cvCreateTrackbar("LowH", "Control Colors", &LowH, 179);
  cvCreateTrackbar("HighH", "Control Colors", &HighH, 179);

  cvCreateTrackbar("LowS", "Control Colors", &LowS, 255);
  cvCreateTrackbar("HighS", "Control Colors", &HighS, 255);

  cvCreateTrackbar("LowV", "Control Colors", &LowV, 255);
  cvCreateTrackbar("HighV", "Control Colors", &HighV, 255);
}
//<<<trackbars for filtering based on color


//>>>image processing from the camera
void find_target_object(int argc, char **argv){

  //>>>ROS initiation, which should be properly called on the first line of the "main program"
  ros::init(argc, argv, "Arm_Camera_Node");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("target_object_coordinates", 1000);
  ros::Rate loop_rate(10);
  //<<<ROS initiation

  //capture video from the camera
  VideoCapture cap(0); //the number here decides which camera to use, 0 for laptop's webcam, 1+ for usb

  //if the capture fails, exit the program
  if(!cap.isOpened()){
    cout<<"ERROR: Cannot open the camera"<<endl;
    exit(EXIT_FAILURE);
  }

  //initiate the trackbars
  createTrackbars();

  //capture a temporary image from the camera to analyze
  cv::Mat imgTmp;
  cap.read(imgTmp);

  //Create a black image with the same size as the camera output
  cv::Mat imgBlack = Mat::zeros(imgTmp.size(), CV_8UC3);;

  while(true){
    Mat original;
    bool boolSuccess = cap.read(original); //read a new frame from the camera
    if(!boolSuccess){//if the the update failed then error
      cout<<"ERROR: Cannot read a new frame from the video stream"<<endl;
      exit(EXIT_FAILURE);;
    }

    //make the frame into HSV from BGR
    cv::Mat imgHSV;
    cv::cvtColor(original, imgHSV, COLOR_BGR2HSV);

    //Threshold the image
    Mat imgThresholded;
    inRange(imgHSV, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), imgThresholded);

    //morphological opening (removes small objects form the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5,5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5,5)));

    //calculate the moments of the thresholded image for info
    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    //if the area is <= 9000, then there are no objects in the frame due to the noise, the area is not zeros
    if(dArea > 9000){

      //calculate the position of the target object
      int posX = dM10 / dArea;
      int posY = dM01 / dArea;

      if (LastX >= 0 && LastY >= 0){
        //Draw a red line from the previous point to the current point
        line(imgBlack, Point(posX, posY), Point(LastX, LastY), Scalar(0,0,255), 2);

        //return the previous frame coordinates of the target object
        //return LastX, LastY;
    }
    else{
      LastX = -1;
      LastY = -1;
      //return LastX, LastY;
      cout<<"ERROR: Target Object not found"<<endl;
    }
      //update the previous coordinates of the target Object
      LastX = posX;
      LastY = posY;
    }

    //show the results
    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    cv::imshow("HSV Image", imgHSV); //show the HSV of the image
    cv::imshow("Original", original); //show the original image
    cv::imshow("Tracking", original + imgBlack); //show the original and the tracking

    //print the coordinates
    //cout<<"X Coordinate: "<<LastX<<", Y Coordinate: "<<LastY<<endl;

    //return LastX;

    //>>>Publish the x coordinate to ros
    std_msgs::Int32 x_coordinate;

    x_coordinate.data = LastX;

    //the publish() function is how you send messages, the parameter is the message object
    //the type of this object must agree with the type given as a template paramter to the advertise<>() call, as was done in the constructor above
    chatter_pub.publish(x_coordinate);

    //anounce that the array was published
    ROS_INFO("Camera Coordinates published: %d", x_coordinate.data);
    ros::spinOnce();

    loop_rate.sleep();
    //<<<Publish the x coordinate to ros


    if (waitKey(30) == 27){ //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
      cout<<"esc key has been pressed by user, exiting and terminated program"<< endl;
      exit(EXIT_FAILURE);;
    }
  }
}
//<<<image processing from the camera


//>>>ROS code
void ROS_Publisher(int argc, char **argv){
  //initiate a node called "Arm_Camera_Node"
  ros::init(argc, argv, "Arm_Camera_Node");

  //this is the main access point to communications with the ROS system, this first one fully initializes this node
  ros::NodeHandle n;

  /*
  -this advertise() function is how ROS knows what to publish on a given topic name
  -this invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing
  -advertise() returns a publisher object which allows you to publish messages on that topic through a call to publish()
  -once all copies of the returned publisher object are destroyed, the topic will be automatically unadvertised
  -this second parameter for advertise() is the size of the message queue used for publishing messages, it's the size of the buffer to keep before throwing away the rest
  */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("target_object_coordinates", 1000);

  ros::Rate loop_rate(10);

  //this is used to count how msgs were sent
  int count = 0;

  while (ros::ok()){
    //this is a message object, stuff it with data, then publish it
    std_msgs::String msg;

    std::stringstream ss;
    ss<<"hello world"<<count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    //the publish() function is how you send messages, the parameter is the message object
    //the type of this object must agree with the type given as a template paramter to the advertise<>() call, as was done in the constructor above
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


}
//<<<ROS code


//>>> int main
int main(int argc, char **argv){//passing argc and argv is needed here to perform any ROS arguments and name remapping given at the command line

  //ROS_Publisher();
  //initiate a node called "Arm_Camera_Node"
  //ros::init(argc, argv, "Arm_Camera_Node");
  //this is the main access point to communications with the ROS system, this first one fully initializes this node

  find_target_object(argc, argv);

  return 0;
}
//<<< int main
