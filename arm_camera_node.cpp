//This Node is for the camera on top of the arm of Fetch
//The camera locates where the target object to fetch is and its coordinates
//The coordinates are then published to a topic to be used by other nodes

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
int LowH = 40;
int HighH = 80;

int LowS = 0;
int HighS = 90;

int LowV = 100;
int HighV = 255;

//Global Variables for the default values of the previous cooridnate of the target object
int LastX = -1;
int LastY = -1;


//User choose target object code>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

bool caliMode;//used for showing debugging windows, trackbars etc.

bool mouseDragging;//used for showing a rectangle on screen as user clicks and drags mouse
bool mouseMove;
bool rectangleSelected;
cv::Point initialClickPt, currentMousePt; //keep track of initial point clicked and current position of mouse
cv::Rect rectangleROI; //this is the ROI that the user has selected
vector<int> H_ROI, S_ROI, V_ROI;// HSV values from the click/drag ROI region stored in separate vectors so that we can sort them easily

void on_trackbar(int, void*){//This function gets called whenever a trackbar position is changed

}

void createTrackbars(){
	//create window for trackbars
	namedWindow(trackbarWindowName, 0);

	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into the window
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, 255, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, 255, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, 255, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, 255, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, 255, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, 255, on_trackbar);
}


void clickDragRect(int event, int x, int y, int flags, void* param){
	//only if calibration mode is true will we use the mouse to change HSV values
	if (caliMode == true){
		//get handle to video feed passed in as "param" and cast as Mat pointer
		Mat* videoFeed = (Mat*)param;

		if (event == CV_EVENT_LBUTTONDOWN && mouseDragging == false){
			//keep track of initial point clicked
			initialClickPt = cv::Point(x, y);
			//user has begun dragging the mouse
			mouseDragging = true;
		}
		/* user is dragging the mouse */
		if (event == CV_EVENT_MOUSEMOVE && mouseDragging == true){
			//keep track of current mouse point
			currentMousePt = cv::Point(x, y);
			//user has moved the mouse while clicking and dragging
			mouseMove = true;
		}
		/* user has released left button */
		if (event == CV_EVENT_LBUTTONUP && mouseDragging == true){
			//set rectangle ROI to the rectangle that the user has selected
			rectangleROI = Rect(initialClickPt, currentMousePt);

			//reset boolean variables
			mouseDragging = false;
			mouseMove = false;
			rectangleSelected = true;
		}

		if (event == CV_EVENT_RBUTTONDOWN){
			//user has clicked right mouse button
			//Reset HSV Values
			H_MIN = 0;
			S_MIN = 0;
			V_MIN = 0;
			H_MAX = 255;
			S_MAX = 255;
			V_MAX = 255;

		}
	}
}


void recordHSV_Values(cv::Mat frame, cv::Mat HSV_Frame){

	//save HSV values for ROI that user selected to a vector
	if (mouseMove == false && rectangleSelected == true){
		//clear previous vector values
		if (H_ROI.size()>0) H_ROI.clear();
		if (S_ROI.size()>0) S_ROI.clear();
		if (V_ROI.size()>0 )V_ROI.clear();
		//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
		if (rectangleROI.width < 1 || rectangleROI.height < 1) cout <<"Error: no rectangle created" << endl;
		else{
			for (int i = rectangleROI.x; i < rectangleROI.x + rectangleROI.width; i++){
				//iterate through both x and y direction and save HSV values at each and every point
				for (int j = rectangleROI.y; j < rectangleROI.y + rectangleROI.height; j++){
					//save HSV value at this point
					H_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[0]);
					S_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[1]);
					V_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[2]);
				}
			}
		}
		//reset rectangleSelected so user can select another region if necessary
		rectangleSelected = false;
		//set min and max HSV values from min and max elements of each array
		if (H_ROI.size() > 0){
			//NOTE: min_element and max_element return iterators so we must dereference them with "*"
			H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
			H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
			cout << "MIN 'H' VALUE: " << H_MIN << endl;
			cout << "MAX 'H' VALUE: " << H_MAX << endl;
		}
		if (S_ROI.size() > 0){
			S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
			S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
			cout << "MIN 'S' VALUE: " << S_MIN << endl;
			cout << "MAX 'S' VALUE: " << S_MAX << endl;
		}
		if (V_ROI.size() > 0){
			V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
			V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
			cout << "MIN 'V' VALUE: " << V_MIN << endl;
			cout << "MAX 'V' VALUE: " << V_MAX << endl;
		}

	}

	if (mouseMove == true){
		//if the mouse is held down, we will draw the click and dragged rectangle to the screen
		rectangle(frame, initialClickPt, cv::Point(currentMousePt.x, currentMousePt.y), cv::Scalar(0, 255, 0), 1, 8, 0);
	}

}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void drawCrosshairsRed(int x, int y, Mat &frame){

	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 0, 255), 2);
	if (y - 25 > 0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 0, 255), 2);
	if (y + 25 < FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 0, 255), 2);
	if (x - 25 > 0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 0, 255), 2);
	if (x + 25 < FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 0, 255), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 0, 255), 2);
}


void drawCrosshairs(int x, int y, Mat &frame){

	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25 > 0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25 < FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25 > 0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25 < FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}


void morphOps(Mat &thresh){
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}


void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int largestIndex = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0){
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]){
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					//save index of largest contour to use with drawContours
					largestIndex = index;
				}
				else objectFound = false;
			}
			//let user know you found an object
			if (objectFound == true){
				putText(cameraFeed, "Target Object Found", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawCrosshairs(x, y, cameraFeed);
				//draw largest contour
				drawContours(cameraFeed, contours, largestIndex, Scalar(0, 0, 255), 2);
        if(y > 270){
          //This is the horizon line
          line(cameraFeed, Point(0, 270), Point(640, 270), Scalar(0, 255, 0), 2);
        }
        if(y > 270 && (x > 160 && x < 480)){
          //This is the left center line
          line(cameraFeed, Point(160, 0), Point(160, 480), Scalar(0, 255, 0), 2);
          //This is the right center line
          line(cameraFeed, Point(480, 0), Point(480, 480), Scalar(0, 255, 0), 2);
          putText(cameraFeed, "Target Object Locked", Point(0, 100), 2, 1, Scalar(0, 0, 255), 2);
          drawCrosshairsRed(x, y, cameraFeed);
        }
			}
		}
		else putText(cameraFeed, "Error: too much noise, change filter", Point(0, 50), 1, 2, Scalar(0, 255, 0), 2);
	}
}
//User choose target object code<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//>>>Euclidian Distance Formula
float distance_formula(float x1, float x2, float y1, float y2){
  float distance;
  distance = sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
  return distance;
}
//<<<Euclidian Distance Formula


//>>>trackbars for filtering based on color
void create_Trackbars(){
  //create window
  namedWindow("Control Colors", CV_WINDOW_AUTOSIZE);
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

  //ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("target_object_coordinates", 1000);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("target_object_coordinates", 10);
  ros::Rate loop_rate(10);
  //<<<ROS initiation

  //capture video from the camera
  VideoCapture cap(1); //the number here decides which camera to use, 0 for laptop's webcam, 1+ for usb

  //if the capture fails, exit the program
  if(!cap.isOpened()){
    cout<<"ERROR: Cannot open the camera"<<endl;
    exit(EXIT_FAILURE);
  }

  //initiate the trackbars
  create_Trackbars();

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
    //cv::imshow("HSV Image", imgHSV); //show the HSV of the image
    cv::imshow("Original", original); //show the original image
    cv::imshow("Tracking", original + imgBlack); //show the original and the tracking

    //print the coordinates
    //cout<<"X Coordinate: "<<LastX<<", Y Coordinate: "<<LastY<<endl;

    //return LastX;

    //>>>Publish the x coordinate to ros
    std_msgs::Int32MultiArray coordinates;

    //for loop to push data into the array
    for(int i = 0; i < 2; i++){
      coordinates.data.push_back(rand() % 255);
      coordinates.data[0] = LastX;
      coordinates.data[1] = LastY;
    }
    //x_coordinate.data = LastX;

    //usleep(1000); //sleep to sync up with the arm_control_node
    //sleep(1);

    //the publish() function is how you send messages, the parameter is the message object
    //the type of this object must agree with the type given as a template paramter to the advertise<>() call, as was done in the constructor above
    chatter_pub.publish(coordinates);

    //anounce that the array was published
    ROS_INFO("Camera Coordinates published: x coordinate: %d, y coordinate: %d", coordinates.data[0], coordinates.data[1]);
    //ROS_INFO("Camera Coordinates published:  y coordinate: %d", coordinates.data[1]);
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
  //find_target_object(argc, argv);

  //>>>ROS initiation, which should be properly called on the first line of the "main program"
  ros::init(argc, argv, "Arm_Camera_Node");
  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("target_object_coordinates", 1000);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("coordinates", 10);
  ros::Rate loop_rate(10);

  std_msgs::Int32MultiArray coordinates;
  //<<<ROS initiation


  //choose target object code
  bool trackObjects = true;
	bool useMorphOps = true;
	caliMode = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0); // +1 for webcam
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//must create a window before setting mouse callback
	cv::namedWindow(windowName);
	//set mouse callback function to be active on "Webcam Feed" window
	//we pass the handle to our "frame" matrix so that we can draw a rectangle to it
	//as the user clicks and drags the mouse
	cv::setMouseCallback(windowName, clickDragRect, &cameraFeed);
	//initiate mouse move and drag to false 
	mouseDragging = false;
	mouseMove = false;
	rectangleSelected = false;

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (true){
		//store image to matrix
		capture.read(cameraFeed);
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
		//set HSV values from user selected region
		recordHSV_Values(cameraFeed, HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);


    //This is the horizon line
    line(cameraFeed, Point(0, 270), Point(640, 270), Scalar(0, 0, 255), 2);
    //This is the left center line
    line(cameraFeed, Point(160, 0), Point(160, 480), Scalar(0, 0, 255), 2);
    //This is the right center line
    line(cameraFeed, Point(480, 0), Point(480, 480), Scalar(0, 0, 255), 2);


		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if (useMorphOps)
			morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if (trackObjects)
			trackFilteredObject(x, y, threshold, cameraFeed);

		//show frames
		if (caliMode == true){
			//create slider bars for HSV filtering
			create_Trackbars();
			imshow(windowName1, HSV);
			imshow(windowName2, threshold);
		}
		else{
			destroyWindow(windowName1);
			destroyWindow(windowName2);
			destroyWindow(trackbarWindowName);
		}
		imshow(windowName, cameraFeed);

    coordinates.data.resize(2); //resize the array to avoid Segmentation fault (core dumped)
    //>>>Publish the x coordinate to ROS
    //for loop to push data into the array
    coordinates.data[0] = x;
    coordinates.data[1] = y;

    //cout<<"X: "<<coordinates.data[0]<<endl;
    //cout<<"Y: "<<coordinates.data[1]<<endl;

    //anounce that the array was published
    ROS_INFO("Camera Coordinates published: x coordinate: %d, y coordinate: %d", coordinates.data[0], coordinates.data[1]);

    chatter_pub.publish(coordinates);
    ros::spinOnce();

    loop_rate.sleep();
    //<<<Publish the x coordinate to ROS


		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//also use waitKey command to capture keyboard input
		if (waitKey(30) == 99) caliMode = !caliMode;//if user presses 'c', toggle calibration mode

    if (waitKey(30) == 27){ //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
      cout<<"esc key has been pressed by user, exiting and terminating program"<< endl;
      exit(EXIT_FAILURE);;
    }
	}
  return 0;
}
//<<< int main
