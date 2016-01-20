/*

Copyright (C) 2014  Kevin Harrilal, Control Engineer, Aluminum Falcon Robotics Inc.
kevin@team2168.org

Dec 31, 2014

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

*/
#define _USE_MATH_DEFINES
#define MIN_WIDTH 120
#define Y_IMAGE_RES 240
#define VIEW_ANGLE 34.8665269
#define AUTO_STEADY_STATE 1.9 //seconds

#include "../Robot.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <pthread.h>

//using namespace cv;
//using namespace std;

//struct to define program execution variables passed in from the command line
struct ProgParams
{
	std::string ROBOT_IP;
	std::string ROBOT_PORT;
	std::string CAMERA_IP = "10.20.53.10";
	std::string IMAGE_FILE = "/home/vision/144_L.jpg";

	bool From_Camera;
	bool From_File;
	bool Visualize;
	bool Timer;
	bool Debug;
	bool Process;
	bool USB_Cam;
};

//Stuct to hold information about targets found
struct Target
{
	cv::Rect HorizontalTarget;
	cv::Rect VerticalTarget;

	double HorizontalAngle;
	double VerticalAngle;
	double Horizontal_W_H_Ratio;
	double Horizontal_H_W_Ratio;
	double Vertical_W_H_Ratio;
	double Vertical_H_W_Ratio;

	cv::Point HorizontalCenter;
	cv::Point VerticalCenter;

	bool HorizGoal;
	bool VertGoal;
	bool HotGoal;
	bool matchStart;
	bool validFrame;

	//camera bool
	bool cameraConnected;

	int targetLeftOrRight;
	int lastTargerLorR;
	int hotLeftOrRight;
	double targetDistance;

};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
void printCommandLineUsage();
void initializeParams(ProgParams& params);
double diffClock(timespec start, timespec end);
cv::Mat ThresholdImage(cv::Mat img);
void findTarget(cv::Mat original, cv::Mat thresholded, Target& targets, const ProgParams& params);
void NullTargets(Target& target);
void CalculateDist(Target& targets);
void error(const char *msg);

//Threaded Video Capture Function
void *VideoCap(void *args);

//GLOBAL CONSTANTS
const double PI = 3.141592653589793;

//Thresholding parameters
int minR = 0;
int maxR = 30;
int minG = 80; //160 for ip cam, 80 to support MS webcam
int maxG = 255;
int minB = 0;
int maxB = 30;

//Target Ratio Ranges
double MinHRatio = 1.5;
double MaxHRatio = 6.6;

double MinVRatio = 1.5;
double MaxVRatio = 8.5;

int MAX_SIZE = 255;

//Some common colors to draw with
const cv::Scalar RED = cv::Scalar(0, 0, 255),
			BLUE = cv::Scalar(255, 0, 0),
			GREEN = cv::Scalar(0, 255, 0),
			ORANGE = cv::Scalar(0, 128, 255),
			YELLOW = cv::Scalar(0, 255, 255),
			PINK = cv::Scalar(255, 0,255),
			WHITE = cv::Scalar(255, 255, 255);

//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;


//Thread Variables
pthread_t MJPEG;
pthread_t AutoCounter;


//Store targets in global variable
Target targets;
cv::Mat frame;

//Global Timestamps for auto
struct timespec autoStart, autoEnd;


//Control process thread exectution
bool progRun;

//testing
int j = 0;

int visionTest()
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	params.From_Camera = true;
	params.From_File = false;
	params.USB_Cam = false;
	params.Visualize = false;
	params.Timer = true;
	params.Process = true;

	//start mjpeg stream thread
	pthread_create(&MJPEG, NULL, VideoCap, &params);

	//Create Local Processing Image Variables
	cv::Mat img, thresholded, output;


	//initialize variables so processing loop is false;
	targets.matchStart = false;
	targets.validFrame = false;
	targets.hotLeftOrRight = 0;
	progRun = false;
	std::ofstream myFile;

	struct timespec start, end;

	//run loop forever
	while (true)
	{
		//check if program is allowed to run
		//this bool, is enabled by the mjpeg thread
		//once it is up to 10fps
		std::cout << "Process: " << params.Process << " progRun: " << progRun << ".\n";
		if (params.Process && progRun)
		{
			//printf("progRun is true and params.Process is true!\n");
			//start clock to determine our processing time;
			clock_gettime(CLOCK_REALTIME, &start);

			pthread_mutex_lock(&frameMutex);
			if (!frame.empty())
			{
				printf("frame is not empty!\n");
				frame.copyTo(img);
				int i = frame.at<uchar>(101,97);
				std::cout << "frame.at: " << i << "\n";
			    std::cout << "rows: " << frame.rows << " cols: " << frame.cols << "\n";
			    if(j == 1) {
			    	 cv::imwrite("/home/lvuser/alpha.jpg", frame);
			    }
			    j++;
				//Image* myImaqImage = imaqCreateImage(IMAQ_IMAGE_RGB,  0);
				//cv::Mat rgba;
				//cv::cvtColor(frame, rgba, CV_BGR2BGRA, 4);
				//int rc = imaqArrayToImage(myImaqImage, rgba.data, img.cols, img.rows);
				//CameraServer::GetInstance()->SetImage(myImaqImage);
				pthread_mutex_unlock(&frameMutex);

				thresholded = ThresholdImage(img);

				//Lock Targets and determine goals
				pthread_mutex_lock(&targetMutex);
				findTarget(img, thresholded, targets, params);
				CalculateDist(targets);

				if(params.Debug)
				{
					std::cout<<"Vert: "<<targets.VertGoal<<std::endl;
					std::cout<<"Horiz: "<<targets.HorizGoal<<std::endl;
					std::cout<<"Hot Goal: "<<targets.HotGoal<<std::endl;
					std::cout<<"Dist:" <<targets.targetDistance<<std::endl<<std::endl;
				}
				pthread_mutex_unlock(&targetMutex);

				clock_gettime(CLOCK_REALTIME, &end);

				if(params.Timer)
					std::cout << "It took " << diffClock(start,end) << " seconds to process frame \n";


			}

			pthread_mutex_unlock(&frameMutex);

			if(params.Visualize)
				cv::waitKey(5);

		}

		usleep(1000); //20000 sleep for 5ms); // run 40 times a second
	}

	//if we end the process code, wait for threads to end
	pthread_join(MJPEG, NULL);

	//done
	return 0;

}

///////////////////FUNCTIONS/////////////////////

/**
 * This function uses the law of lense projection to
 * estimate the distance to an object of known height only
 * using a single camera.
 *
 * This function uses only the vertical target height, the
 * pixel height of the image, and the view angle of the
 * camera lense.
 */
void CalculateDist(Target& targets)
{
	//vertical target is 32 inches fixed
	double targetHeight = 32.0;

	//get vertical pixels from targets
	int height = targets.VerticalTarget.height;

	//d = Tft*FOVpixel/(2*Tpixel*tanΘ)
	targets.targetDistance = Y_IMAGE_RES * targetHeight
			/ (height * 12 * 2 * tan(VIEW_ANGLE * PI / (180 * 2)));
}

/**
 * This function scans through an image and determins
 * if rectangles exist which match the profile of a
 * "Hot Goal".
 *
 * The "Hot Goal" is specific to the 2014 FRC game
 * and is identified as a horizontal and vertical target
 * in the same frame, with known width and height.
 */
void findTarget(cv::Mat original, cv::Mat thresholded, Target& targets, const ProgParams& params)
{

	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<Point> > contours;
	std::vector<int> selected;
	std::vector<cv::Rect> rectangle;

	cv::findContours(thresholded, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE); // finds contours

	std::vector<Rect> boundRect(contours.size()); //vector of bounding rectangles

	cv::Mat drawing = cv::Mat::zeros(thresholded.size(), CV_8UC3); //empty mat for drawing on

	for (int i = 0; i < contours.size(); i++) //loops through all contours and checks if they are big enough, if so makes a bounding box
	{
		cv::Scalar color = cv::Scalar(0, 255, 0);
		cv::Rect R = cv::boundingRect(contours[i]);
		// filter contours according to their bounding box
		if (R.area() >= 300)
		{
			selected.push_back(i);
			cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		}
	}

	for (size_t i = 0; i < selected.size(); i++) //draws rectangles
	{
		cv::rectangle(drawing, cv::boundingRect(contours[selected[i]]), cv::Scalar(0, 0, 255), 5);
	}
}

/**
 * This function performs numerous filtering on
 * a color image in order to only return
 * areas of interest based on their color
 *
 */
cv::Mat ThresholdImage(cv::Mat original)
{
	//Local Temp Image
	cv::Mat thresholded;

	//Threshold image to remove non-green objects
	cv::inRange(original, cv::Scalar(minB, minG, minR), cv::Scalar(maxB, maxG, maxR),
			thresholded);

	//smooth edges
	cv::blur(thresholded, thresholded, cv::Size(3, 3));

	//Additional filtering if needed
	//Canny(thresholded, thresholded, 100, 100, 3);
	//blur(thresholded, thresholded, Size(5, 5));

	//return image
	return thresholded;

}

/**
 * This functions "zeros", the targets identified
 * so that a clean slate can be used to determine
 *] if the next image contains targets as well.
 */
void NullTargets(Target& target)
{

	target.HorizontalAngle = 0.0;
	target.VerticalAngle = 0.0;
	target.Horizontal_W_H_Ratio = 0.0;
	target.Horizontal_H_W_Ratio = 0.0;
	target.Vertical_W_H_Ratio = 0.0;
	target.Vertical_H_W_Ratio = 0.0;
	target.targetDistance = 0.0;
	target.targetLeftOrRight = 0;
	target.lastTargerLorR = 0;

	target.HorizGoal = false;
	target.VertGoal = false;
	target.HotGoal = false;

}
void initializeParams(ProgParams& params)
{
	params.Debug = false;
	params.From_Camera = false;
	params.From_File = false;
	params.Timer = false;
	params.Visualize = false;
	params.Process = true;
	params.USB_Cam = false;

}

/**
 * This function parses the command line inputs and determines
 * the runtime parameters the program should use as specified
 * by the user.
 */
void parseCommandInputs(int argc, const char* argv[], ProgParams& params)
{
	//todo: define all input flags
	if (argc < 2)
	{ // Check the value of argc. If not enough parameters have been passed, inform user and exit.
		printCommandLineUsage();
		exit(0);
	}
	else
	{ // if we got enough parameters...

		initializeParams(params);

		for (int i = 1; i < argc; i++)
		{ /* We will iterate over argv[] to get the parameters stored inside.
		 * Note that we're starting on 1 because we don't need to know the
		 * path of the program, which is stored in argv[0] */

			if ((std::string(argv[i]) == "-f") && (i + 1 < argc)) //read from file
			{
				// We know the next argument *should* be the filename:
				params.IMAGE_FILE = std::string(argv[i + 1]);
				params.From_Camera = false;
				params.From_File = true;
				i++;
			}
			else if ((std::string(argv[i]) == "-c") && (i + 1 < argc)) //camera IP
			{
				params.CAMERA_IP = std::string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = false;
				i++;
			}
			else if (std::string(argv[i]) == "-u") //use USB Camera
			{
				//params.CAMERA_IP = std::string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = true;
			}
			else if ((std::string(argv[i]) == "-s") && (i + 1 < argc)) //robot TCP SERVER IP
			{
				params.ROBOT_IP = std::string(argv[i + 1]);
				i++;
			}
			else if ((std::string(argv[i]) == "-p") && (i + 1 < argc)) //robot TCP SERVER PORT
			{
				params.ROBOT_PORT = std::string(argv[i + 1]);
				i++;
			}
			else if (std::string(argv[i]) == "-t") //Enable Timing
			{
				params.Timer = true;
			}
			else if (std::string(argv[i]) == "-np") //no processing
			{
				params.Process = false;
			}
			else if (std::string(argv[i]) == "-v") //Enable Visual output
			{
				params.Visualize = true;
			}
			else if (std::string(argv[i]) == "-debug") //Enable debug output
			{
				params.Debug = true;
			}
			else if (std::string(argv[i]) == "-d") //Default Params
			{
				params.ROBOT_PORT = std::string(argv[i + 1]);
				return;
			}
			else if (std::string(argv[i]) == "-help") //help
			{
				//todo: std::cout help on commands
				printCommandLineUsage();
				exit(0);
			}
			else
			{
				std::cout
						<< "Not enough or invalid arguments, please try again.\n";
				printCommandLineUsage();
				exit(0);
			}

		}

	}
}


/**
 * This function uses FFMPEG codec apart of openCV to open a
 * MJPEG stream and buffer it. This function should be ran
 * in its own thread so it can run as fast as possibe and store frames.
 *
 * A mutable lock should be used in another thread to copy the latest frame
 *
 * Note: Opening the stream blocks execution. Also
 * Based on my own tests it appears the beaglebone can capture
 * frames at 30fps with 320 x 240 resolution, however
 * the framerate needs to be reduced to allow for processing time.
 *
 * Only run the camera as 10FPS, with a 10kbs limit per frame
 */
void *VideoCap(void *args)
{
	//copy passed in variable to programStruct
	ProgParams *struct_ptr = (ProgParams *) args;

	if (struct_ptr->From_File)
	{
		std::cout<<"Loading Image from file"<<std::endl;

		//read img and store it in global variable
		pthread_mutex_lock(&frameMutex);
		frame = cv::imread(struct_ptr->IMAGE_FILE);
		pthread_mutex_unlock(&frameMutex);

		if (!frame.empty())
		{
			std::cout<<"File Loaded: Starting Processing Thread"<<std::endl;
			progRun = true;
		}
		else
		{
			std::cout<<"Error Loading File"<<std::endl;
			exit(0);
		}


	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 12;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);

		cv::VideoCapture vcap;


		// For IP cam this works on a AXIS M1013
		// For USB cam this works on Microsoft HD 3000


		std::string videoStreamAddress;
		if (struct_ptr->USB_Cam)
		{

			int videoStreamAddress = 0; //represents /dev/video0

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count =1;

			//open the video stream and make sure it's opened
			//We specify desired frame size and fps in constructor
			//Camera must be able to support specified framesize and frames per second
			//or this will set camera to defaults
			while (!vcap.open(videoStreamAddress, 640,480,7.5))
			{
				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

			//After Opening Camera we need to configure the returned image setting
			//all opencv v4l2 camera controls scale from 0.0 - 1.0

			//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
			vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
			vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
			vcap.set(CV_CAP_PROP_CONTRAST, 0);

			std::cout<<vcap.get(CV_CAP_PROP_FRAME_WIDTH)<<std::endl;
			std::cout<<vcap.get(CV_CAP_PROP_FRAME_HEIGHT)<<std::endl;

		}
		else //connect to IP Cam
		{
			std::string videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpg/video.mjpg";

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count = 1;

			//open the video stream and make sure it's opened
			//image settings, resolution and fps are set via axis camera webpage
			while (!vcap.open(videoStreamAddress))
			{

				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

		}



		//Stream started
		std::cout << "Successfully connected to Camera Stream" << std::endl;

		//set true boolean
		pthread_mutex_lock(&targetMutex);
		targets.cameraConnected = true;
		pthread_mutex_unlock(&targetMutex);

		//end clock to determine time to setup stream
		clock_gettime(CLOCK_REALTIME, &end);

		std::cout << "It took " << diffClock(start,end) << " seconds to set up stream " << std::endl;

		clock_gettime(CLOCK_REALTIME, &bufferStart);


		std::cout<<"Waiting for stream buffer to clear..."<<std::endl;


		//run in continuous loop
		while (true)
		{
			//start timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &start);

			//read frame and store it in global variable
			pthread_mutex_lock(&frameMutex);
			vcap.read(frame);
			pthread_mutex_unlock(&frameMutex);

			//end timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &end);


			if(struct_ptr->Timer)
				std::cout << "It took FFMPEG " << diffClock(start,end) << " seconds to grab stream \n";


			//end timer to get time since stream started
			clock_gettime(CLOCK_REALTIME, &bufferEnd);
			double bufferDifference = diffClock(bufferStart, bufferEnd);

			//The stream takes a while to start up, and because of it, images from the camera
			//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
			//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
			//before we are at the end of the stream, and can allow processing to begin.
			if ((bufferDifference >= waitForBufferToClear) && !progRun)
			{
				std::cout<<"Buffer Cleared: Starting Processing Thread"<<std::endl;
				progRun = true;

			}
			usleep(1000); //sleep for 5ms
		}

	}

	return NULL;
}

/*
 * This function prints the command line usage of this
 * program to the std output
 */
void printCommandLineUsage()
{
	std::cout<<"Usage: 2168_Vision  [Input]  [Options] \n\n";

	std::cout<<std::setw(10)<<std::left<<"Inputs:  Choose Only 1"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-f <file location>";
	std::cout<<"Process image at <file location>"<<std::endl;
	std::cout<<std::setw(30)<<""<<"ex: -f /home/image.jpg"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-c <ip address>";
	std::cout<<"Use IP camera at <ip address>"<<std::endl;
	std::cout<<std::setw(30)<<""<<"ex: -c 10.21.68.2"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-u";
	std::cout<<"Use USB cam at /dev/video0"<<std::endl;

	std::cout<<std::endl<<std::endl;
	std::cout<<std::setw(10)<<std::left<<"Options:  Choose Any Combination"<<std::endl;


	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-t";
	std::cout<<"Enable Timing Print Outs"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-v";
	std::cout<<"Enable Visual Output"<<std::endl;
	std::cout<<std::setw(30)<<""<<"Uses X11 forwarding to show processed image"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-np";
	std::cout<<"No Processing: Disable Processing Thread"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-debug";
	std::cout<<"Enable Debug Print Outs"<<std::endl;

	std::cout<<std::setw(10)<<std::left<<"";
	std::cout<<std::setw(20)<<std::left<<"-help";
	std::cout<<"Prints this menu"<<std::endl;


}

/*
 * Error Functions
 * - Not Used -
 */
void error(const char *msg)
{
	perror(msg);
	exit(0);
}

/*
 * Calculate real clock difference
 */
double diffClock(timespec start, timespec end)
{
 return	(end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec)/ 1000000000.0f;
}



