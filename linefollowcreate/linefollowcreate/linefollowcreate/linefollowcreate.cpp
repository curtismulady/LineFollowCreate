// opencv_test_pgm.cpp : main project file.

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"
#include "common.h"

//#define USE_ROBOT_SERIAL_PORT
#define SHOW_SCREENS



using namespace System;
using namespace cv;
using namespace std;


CvCapture* cap = NULL;
Mat camframe,camframecopy,invframe,blurframe,cannyframe,houghframe,greyframe;
Mat origframe,colorfiltframe,colorfilt_linedet_frame;
IplImage* iplimg;
COLORNAME_t colorMode = COLOR_BLUE;
float threshVal = 0.5;



#ifdef USE_ROBOT_SERIAL_PORT
CreateSerial robot(COMNUM);
#endif


int main(array<System::String ^> ^args)
{
    Console::WriteLine(L"Starting...");

	#ifdef USE_ROBOT_SERIAL_PORT
	//Start Robot
	robot.StartRobot();
	robot.SetMode_Full();
	#endif

	Console::WriteLine(L"ready?\n");
	
	


	cap = cvCaptureFromCAM(0);
	if(!cap) printf ("No camera detected.");

	iplimg = cvQueryFrame(cap);
	camframe = iplimg;


	if(iplimg->origin == IPL_ORIGIN_TL)
		camframe.copyTo(camframecopy);
	else
		flip (camframe,camframecopy,0);
	imshow("frame",camframecopy);
	waitKey(0);




	while(cap)
	{
		iplimg = cvQueryFrame(cap);
		camframe = iplimg;

		if(camframe.empty()) break;
		if(iplimg->origin == IPL_ORIGIN_TL)
			camframe.copyTo(camframecopy);
		else
			flip (camframe,camframecopy,0);

		// apply filters
		camframecopy.copyTo(origframe);											//get a new frame to work with
		GaussianBlur(origframe,blurframe,cv::Size(11,11),2.3,0);				//blur the frame
		filter_Color(blurframe,colorfilt_linedet_frame, colorMode, threshVal);	//filter frame for color - turns to float
	

		int lineloc = LineFollow_getDeltaLineLoc(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, 40);
		Draw_LineFollow_LineLoc(colorfilt_linedet_frame,colorfilt_linedet_frame,colorfilt_linedet_frame.rows-20, 40);		
		//printf("%d: ",lineloc);
		//(lineloc>0)?printf("Go Right\n") : printf("Go Left\n");

		if(lineloc != 0xFFFF)
		{
			//follow line
			//printf("R=%d: L=%d\n",50-lineloc,50+lineloc);
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(80-lineloc/CORRECTION_DIV,80+lineloc/CORRECTION_DIV);
			#endif
		}else{
			//find line
			//printf("Oh,crap! Lost the line.\n");
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(80,-80);
			#endif

		}

		printf("solid color green: %f\n\n",getIsFullScreenColor(origframe,COLOR_GREEN,0.5));



		

		#ifdef SHOW_SCREENS
		imshow("frame",camframecopy);
		imshow("floatframe",colorfilt_linedet_frame);
		#endif

		//End of this loop - what to do next?
		int ret = waitKey(1);											//Nothing, keep going
		if( ret == 32 || ret == 13 )break;								// ENTER or SPACE key - end program
		if( ret == 98 ) {colorMode = COLOR_BLUE; threshVal = 0.47;}		// 'b' key - follow blue lines
		if( ret == 103 ) {colorMode = COLOR_GREEN; threshVal = 0.5;}	// 'g' key - follow green lines
		if( ret == 114 ) {colorMode = COLOR_RED; threshVal=0.67;}		// 'r' key - follow red lines
		if( ret == 107 ) {colorMode = COLOR_BLACK; threshVal=0.10;}		// 'k' key - follow black lines




	}
	
	printf("\nStopping Motors\n");
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.DriveMotorDirect(0,0);
	#endif



    return 0;
}




//OLD
/*
Canny(blurframe,cannyframe,10,100,3);
//Get Hough Lines
cv::HoughLines(cannyframe,lines,1,__PI__/180,60.0);
it = lines.begin();

while(it!=lines.end())
{
	float rho = (*it)[0]; 
	float theta = (*it)[1]; 
	DrawRhoThetaLine(camframe, rho, theta, cv::Scalar(255, 0, 0), 1);
	it++;
}
imshow("canny",cannyframe);
*/