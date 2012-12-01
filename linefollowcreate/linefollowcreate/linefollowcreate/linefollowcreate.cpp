// linefollowcreate.cpp : main project file.

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"
#include "common.h"


//===System Specific Defines===

//#define USE_ROBOT_SERIAL_PORT
#define SHOW_SCREENS
#define LINE_DELTA_THRESH_U8 30
#define COMNUM 1
#define ROBOT_SPEED_BASE 80
#define CORRECTION_DIV 7

//=============================

using namespace System;
using namespace cv;
using namespace std;


//Global Variables
CvCapture* cap = NULL;
Mat camframe,camframecopy,invframe,blurframe,cannyframe,houghframe,greyframe;
Mat origframe,colorfiltframe,colorfilt_linedet_frame;
IplImage* iplimg;
COLORNAME_t colorMode = COLOR_BLUE;
float threshVal = 0.5;


//Robot's Serial Connection
#ifdef USE_ROBOT_SERIAL_PORT
CreateSerial robot(COMNUM);
#endif


int main(array<System::String ^> ^args)
{
	//Notify user that program is about to start
    Console::WriteLine(L"Starting...");

	//Start Robot - Initialize 
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.StartRobot();
	robot.SetMode_Full();
	#endif

	//Query user: are you ready>
	Console::WriteLine(L"Ready?\n");	

	//Setup Camera Connection
	cap = cvCaptureFromCAM(0);
	if(!cap) printf ("No camera detected.");
	iplimg = cvQueryFrame(cap);
	camframe = iplimg;

	//Get First Frame from Camera
	if(iplimg->origin == IPL_ORIGIN_TL)
		camframe.copyTo(camframecopy);
	else
		flip (camframe,camframecopy,0);
	imshow("frame",camframecopy);
	waitKey(0);




	while(cap)
	{
		//Get Frame from Camera
		iplimg = cvQueryFrame(cap);
		camframe = iplimg;
		if(camframe.empty()) break;
		if(iplimg->origin == IPL_ORIGIN_TL)
			camframe.copyTo(camframecopy);
		else
			flip (camframe,camframecopy,0);

		//Apply Filters
		camframecopy.copyTo(origframe);											//get a new frame to work with
		GaussianBlur(origframe,blurframe,cv::Size(11,11),2.3,0);				//blur the frame
		filter_Color(blurframe,colorfilt_linedet_frame, colorMode, threshVal);	//filter frame for color - turns to float
	

		//Detect line location and width
		int lineloc = LineFollow_getDeltaLineLoc(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8);
		int line_width = LineFollow_getLineWidth(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) ;
		Draw_LineFollow_LineLoc(colorfilt_linedet_frame,colorfilt_linedet_frame,colorfilt_linedet_frame.rows-20, LINE_DELTA_THRESH_U8);		
		
		//Drive Robot
		if(lineloc != 0xFFFF)
		{
			//follow line
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(ROBOT_SPEED_BASE-lineloc/CORRECTION_DIV,ROBOT_SPEED_BASE+lineloc/CORRECTION_DIV);
			#endif
		}else{
			//find line
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(ROBOT_SPEED_BASE,-ROBOT_SPEED_BASE);
			#endif

		}

		//======Debug Code======
		//printf("solid color green: %f\n\n",getIsFullScreenColor(origframe,COLOR_GREEN,0.5));
		printf("line width: %d\n\n",LineFollow_getLineWidth(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) );
		//printf("%d: ",lineloc);
		//(lineloc>0)?printf("Go Right\n") : printf("Go Left\n");


		
		//Display Screens
		#ifdef SHOW_SCREENS
		imshow("frame",camframecopy);
		#endif
		imshow("floatframe",colorfilt_linedet_frame);

		//End of this loop - what to do next?
		int ret = waitKey(1);											//Nothing, keep going
		if( ret == 32 || ret == 13 )break;								// ENTER or SPACE key - end program
		if( ret == 98 ) {colorMode = COLOR_BLUE; threshVal = 0.47;}		// 'b' key - follow blue lines
		if( ret == 103 ) {colorMode = COLOR_GREEN; threshVal = 0.5;}	// 'g' key - follow green lines
		if( ret == 114 ) {colorMode = COLOR_RED; threshVal=0.67;}		// 'r' key - follow red lines
		if( ret == 107 ) {colorMode = COLOR_BLACK; threshVal=0.10;}		// 'k' key - follow black lines

	}
	
	//Stop Robot
	printf("\nStopping Motors\n");
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.DriveMotorDirect(0,0);
	#endif


	//End of Line
    return 0;
}


