#include <opencv2/opencv.hpp>
#include <math.h>
#include <ctime>
#include "../Robot.h"

#define PI 3.14159265

//	constants for the color rbg values
	const cv::Scalar RED = cv::Scalar(0, 0, 255),
	BLUE = cv::Scalar(255, 0, 0),
	GREEN = cv::Scalar(0, 255, 0),
	BLACK = cv::Scalar(0,0,0),
	YELLOW = cv::Scalar(0, 255, 255),
//	these are the threshold values in order
	LOWER_BOUNDS = cv::Scalar(239,0,109),
	UPPER_BOUNDS = cv::Scalar(300,255,240);

//	the size for resing the image
	const cv::Size resize = cv::Size(320,240);

//	ignore these
	 cv::VideoCapture videoCapture;
	 cv::Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;

//	Constants for known variables
//	the height to the top of the target in first stronghold is 97 inches
	 const int TOP_TARGET_HEIGHT = 97;
//	the physical height of the camera lens
	 const int TOP_CAMERA_HEIGHT = 32;

//	camera details, can usually be found on the datasheets of the camera
	 const double VERTICAL_FOV  = 51;
	 const double HORIZONTAL_FOV  = 67;
	 const double CAMERA_ANGLE = 10;

	 bool shouldRun = true;

	 int biggestArea = 0;
	 int biggestAreaIndex = 0;

	 bool buttonPressed = false;

	 std::vector<std::vector<cv::Point>> contours;
	 std::vector<std::vector<cv::Point>> selected;

/**
 * @param angle a nonnormalized angle
 */
 double normalize360(double angle){
	while(angle >= 360.0)
	{
		angle -= 360.0;
	}
	while(angle < 0.0)
	{
		angle += 360.0;
	}
	return angle;
}



 /**
  *
  * reads an image from a live image capture and outputs information to the SmartDashboard or a file
  */
 void processImage(){
	//buttonPressed = Robot::oi->getDriveJoystick()->GetRawButton(5);
	printf("IM AM IN PROCESS IMAGE!\n");
 	double x,y,targetX,targetY,distance,azimuth;
 //		frame counter
 	int FrameCount = 0;
 //		only run for the specified time
 	while(FrameCount < 100){
 		std::cout << "frameCount: " << FrameCount << std::endl;
 		contours.clear();
 		selected.clear();
 //			capture from the axis camera
 		printf("before read\n");
 		videoCapture.read(matOriginal);
 		printf("after read\n");
 //			captures from a static file for testing
 		//matOriginal = cv::imread("/home/lvuser/original.png");
 		cv::imwrite("/home/lvuser/original.jpg", matOriginal);
 		cv::cvtColor(matOriginal,matHSV,cv::COLOR_BGR2HSV);
 		cv::inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
 		cv::findContours(matThresh, contours, matHeirarchy, cv::RETR_EXTERNAL,
 				cv::CHAIN_APPROX_SIMPLE);
 		std::cout << "Size of contours: " << contours.size() << std::endl;
 //			make sure the contours that are detected are at least 20x20
 //			pixels with an area of 400 and an aspect ration greater then 1
 		//printf("looping\n");
 		for (unsigned int i = 0; i < contours.size();  i++) {
 			cv::Rect rec = cv::boundingRect(contours[i]);
			float aspect = (float)rec.width/(float)rec.height;
			if(aspect > 1.0) {
				if(rec.area() > biggestArea)
					biggestArea = rec.area();
					selected.push_back(contours[i]);
				}
 		}
 		std::cout << "Size of selected: " << selected.size() << std::endl;
 		for(unsigned int i = 0; i < selected.size(); i++){
 			cv::Rect rec = cv::boundingRect(selected[i]);
 			std::cout << "looping on selected!" << std::endl;
 			cv::rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
 		}
 //			if there is only 1 target, then we have found the target we want
 		if(selected.size() == 1){
 			std::cout << "selected is one!" << std::endl;
 			cv::Rect rec = cv::boundingRect(selected[0]);
 //				"fun" math brought to you by miss daisy (team 341)!
 			y = rec.br().y + rec.height / 2;
 			y= -((2 * (y / matOriginal.cols)) - 1);
 			distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) /
 					tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * PI / 180);
 //				angle to target...would not rely on this
 			targetX = rec.tl().x + rec.width / 2;
 			targetX = (2 * (targetX / matOriginal.rows)) - 1;
 			azimuth = normalize360(targetX*HORIZONTAL_FOV /2.0 + 0);
 //				drawing info on target
 			cv::Point center =  cv::Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2);
 			cv::Point centerw =  cv::Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2 - 20);
 			cv::putText(matOriginal, ""+(int)distance, center, cv::FONT_HERSHEY_PLAIN, 1, BLACK);
 			cv::putText(matOriginal, ""+(int)azimuth, centerw, cv::FONT_HERSHEY_PLAIN, 1, BLACK);
 		}
 //			output an image for debugging
 	 		cv::imwrite("/home/lvuser/output.jpg", matOriginal);
 		FrameCount++;
 	}
 	//shouldRun = false;
}

/**
 *
 * @param args command line arguments
 * just the main loop for the program and the entry points
 */
 void visionTest() {
	// TODO Auto-generated method stub
	matOriginal =  cv::Mat();
	matHSV =  cv::Mat();
	matThresh =  cv::Mat();
	clusters =  cv::Mat();
	matHeirarchy =  cv::Mat();
//		main loop of the program
	while(shouldRun){
		try {
//				opens up the camera stream and tries to load it
			videoCapture =  cv::VideoCapture();
//				replaces the ##.## with your team number
			videoCapture.open("http://axis-camera.local/mjpg/video.mjpg");
//				Example
//				cap.open("http://10.30.19.11/mjpg/video.mjpg");
//				wait until it is opened
			while(!videoCapture.isOpened()){
				printf("videoCapture could not open!\n");
			}
//				time to actually process the acquired images
			processImage();
		} catch (std::exception& e) {
			std::cout << "MyException caught" << std::endl;
			std::cout << e.what() << std::endl;
			break;
		}
	}
//		make sure the java process quits when the loop finishes
	videoCapture.release();
	exit(0);
}
