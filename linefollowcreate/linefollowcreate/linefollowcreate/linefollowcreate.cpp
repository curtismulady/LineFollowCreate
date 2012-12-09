// linefollowcreate.cpp : main project file.

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"
#include "common.h"
#include <ctime>
#include <tchar.h>

using namespace System;
using namespace cv;
using namespace std;


//===System Specific Defines======

#define USE_ROBOT_SERIAL_PORT
#define RUN_STATE_MACHINE
#define SHOW_SCREENS
//#define MAKE_LOG_FILE

#define LINE_DELTA_THRESH_U8 30
#define COMNUM 1
#define ROBOT_SPEED_BASE 70
#define CORRECTION_DIV 3

//================================


//Global OPENCV Variables
CvCapture* cap = NULL;
Mat camframe,camframecopy,blurframe,greyframe;
Mat origframe;
Mat colorfiltframe,colorfilt_linedet_frame;
IplImage* iplimg;

//System Variables
COLORNAME_t colorMode = COLOR_BLUE;
float threshVal = 0.39;
int lineloc,lineloc_last,line_width;
int ret=0;//return key
int countdown = 0;
int frame_counter = 0;

//Logfile Variables
HANDLE logfile;
char DataBuffer[500];
DWORD dwBytesToWrite = 0;
DWORD dwBytesWritten = 0;
BOOL bErrorFlag = FALSE;

//State Machine Enumeration
typedef enum {
				ROBOT_STATE_Init='i',
				ROBOT_STATE_WaitForIDScan='w',
				ROBOT_STATE_FindLineToGoal='g',
				ROBOT_STATE_FollowLineToGoal = '>',
				ROBOT_STATE_DrivePastGoal='G',
				ROBOT_STATE_WaitForReturnCommand = 'r',
				ROBOT_STATE_FindLineToHome = 'h',
				ROBOT_STATE_FollowLineToHome = '<',
				ROBOT_STATE_DrivePastHome = 'H'
			} RobotState_t;
RobotState_t RobotState = ROBOT_STATE_Init;

//Initialize Robot's Serial Connection
#ifdef USE_ROBOT_SERIAL_PORT
CreateSerial robot(COMNUM);
#endif

//===========MAIN ENTRY POINT=========
int main(array<System::String ^> ^args)
{
	//Notify user that program is about to start
    Console::WriteLine(L"Starting...");

	//Start Robot - Initialize Robot
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.StartRobot();
	robot.SetMode_Full();
	#endif

	//Create and start a log file
	#ifdef MAKE_LOG_FILE
	logfile = CreateFile(TEXT("./log.txt"),GENERIC_WRITE,0,NULL,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,NULL);
	if (logfile == INVALID_HANDLE_VALUE) 
	{
		printf("Logfile could not be created.\n");
		return -1;
	}
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	strftime (buffer,80,"%m-%d-%Y  %H:%M:%S",timeinfo);
	sprintf(DataBuffer,"Logfile created: %s\n\n\n",buffer);
	dwBytesToWrite = (DWORD)strlen(DataBuffer);
	bErrorFlag = WriteFile( logfile, DataBuffer, dwBytesToWrite, &dwBytesWritten, NULL);   
	#endif

	//Query user: are you ready?
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

	//Hold for keypress (allows setup with monitor prior to running loop)
	waitKey(0);

	//=========MAIN LOOP============
	while(cap)
	{
		//Inc framecounter
		frame_counter++;
				
		//Write Data to logfile
		#ifdef MAKE_LOG_FILE
		sprintf(DataBuffer,"Frame No.: %d\n",frame_counter);
		dwBytesToWrite = (DWORD)strlen(DataBuffer);
		bErrorFlag = WriteFile( logfile, DataBuffer, dwBytesToWrite, &dwBytesWritten, NULL);   
		#endif

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
		filter_Color(blurframe,colorfilt_linedet_frame, colorMode, threshVal);	//filter frame for color


		//Detect line location and width
		if(lineloc != 0xFFFF && line_width < 500 && line_width > 0  && lineloc < 300 && lineloc > -300) lineloc_last = lineloc;
		lineloc = LineFollow_getDeltaLineLoc(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8);
		line_width = LineFollow_getLineWidth(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) ;
		Draw_LineFollow_LineLoc(colorfilt_linedet_frame,colorfilt_linedet_frame,colorfilt_linedet_frame.rows-20, LINE_DELTA_THRESH_U8);		
		printf("Line: %04d | LineLast: %04d | LineWidth : %04d\n",lineloc,lineloc_last,line_width);
		
		//Write Data to logfile
		#ifdef MAKE_LOG_FILE
		sprintf(DataBuffer,"\tEntering State: %c\n",(char)RobotState);
		dwBytesToWrite = (DWORD)strlen(DataBuffer);
		bErrorFlag = WriteFile( logfile, DataBuffer, dwBytesToWrite, &dwBytesWritten, NULL);   
		#endif

#ifdef RUN_STATE_MACHINE
		//=========ROBOT STATE MACHINE================
		printf("State=");
		switch(RobotState)
		{

		//Init - wait for enter key press, then beep 3x and move to WaitForIDScan
		case ROBOT_STATE_Init:
			printf("Init");
			if(ret == 32 || ret == 13)
			{
				RobotState = ROBOT_STATE_WaitForIDScan;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.TripleBeep();
				countdown = 0;
				#endif
			}
			break;

		//WaitForIDScan - wait for color to fill up screen (using ID card)
		//Once ID validated, beep, and flush some frames through before starting a run for the goal
		case ROBOT_STATE_WaitForIDScan:
			printf("WaitForIDScan");
			
			if(getIsFullScreenColor(blurframe, COLOR_BLUE, 0.39) > 0.7)
			{
				colorMode = COLOR_BLUE; threshVal = 0.39;
				RobotState = ROBOT_STATE_FindLineToGoal;
				lineloc = 0; lineloc_last = 0; line_width = 1;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.Beep();
				#endif
				Sleep(2000);
				//Go through a few frames, then move on
				countdown = 5;
			}
			if(getIsFullScreenColor(blurframe, COLOR_GREEN, 0.5) > 0.7)
			{
				colorMode = COLOR_GREEN; threshVal = 0.5;
				RobotState = ROBOT_STATE_FindLineToGoal;
				lineloc = 0; lineloc_last = 0; line_width = 1;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.Beep();
				#endif
				Sleep(2000);
				//Go through a few frames, then move on
				countdown = 5;
			}
			if(getIsFullScreenColor(blurframe, COLOR_RED, 0.67) > 0.7)
			{
				colorMode = COLOR_RED; threshVal = 0.67;
				lineloc = 0; lineloc_last = 0; line_width = 1;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.Beep();
				#endif
				Sleep(2000);
				//Go through a few frames, then move on
				countdown = 5;
			}
			//We want to get the ext few camera frames before moving on
			if(countdown > 1)
			{
				countdown--;
			}
			if (countdown == 1)
			{
				RobotState = ROBOT_STATE_FindLineToGoal;
				countdown = 0;
			}
			break;

		//FindLineToGoal - Robot follows specified line. If line cannot be found, robot will turn in search for it.
		//Robot beeps low tone when it thinks it has found home
		case ROBOT_STATE_FindLineToGoal:
			printf("FindLineToGoal");			
			//printf("BLACK: Line: %04d | LineLast: %04d | LineWidth : %04d\n",blacklineloc,blacklineloc_last,blackline_width);
			
			if(line_width > 400 && line_width < 600)
			{
				printf("\nGOt It?\n");
				RobotState = ROBOT_STATE_DrivePastGoal;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.DoubleBeep();
				#endif
			}
			
			if(lineloc != 0xFFFF && line_width < 500 && line_width > 0  && lineloc < 300 && lineloc > -300)
			{
				//havent found goal yet - follow line
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.DriveMotorDirect(ROBOT_SPEED_BASE-lineloc/CORRECTION_DIV,ROBOT_SPEED_BASE+lineloc/CORRECTION_DIV);
				#endif
			}else{
				//if lost - turn until we find the line again
				if(lineloc_last < 0)
				{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(ROBOT_SPEED_BASE,-ROBOT_SPEED_BASE);
					#endif
				}else{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(-ROBOT_SPEED_BASE,ROBOT_SPEED_BASE);
					#endif
				}
			}
			break;

		//FollowLineToGoal - not used - this is summed up in the previous state
		case ROBOT_STATE_FollowLineToGoal:
			printf("FollowLineToGoal");
			break;

		//DrivePastGoal - Automatic behavior to situate robot over goal and make a 180 turn
		case ROBOT_STATE_DrivePastGoal:
			printf("DrivePastGoal");
			//Perform 'move forward' maneuver
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(ROBOT_SPEED_BASE,ROBOT_SPEED_BASE);
			#endif
			ret = waitKey(4000);
			//If user says this is not a goal - keep chuggin.
			if(ret == 'n') {RobotState = ROBOT_STATE_FindLineToGoal; break;}
			//Perform 'turn 180' maneuver
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(-ROBOT_SPEED_BASE*2,ROBOT_SPEED_BASE*2);
			#endif
			ret = waitKey(3000);
			//Halt
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(0,0);
			#endif
			RobotState = ROBOT_STATE_WaitForReturnCommand;

			break;

		//WaitForReturnCommand - sit and do nothing until told otherwise (enter key)
		case ROBOT_STATE_WaitForReturnCommand:
			printf("WaitForReturnCommand");
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(0,0);
			#endif
			if(ret == 32 || ret == 13)
			{
				RobotState = ROBOT_STATE_FindLineToHome;
			}
			break;

		//FindLineToHome - Robot follows specified line. If line cannot be found, robot will turn in search for it.
		//Robot beeps low tone when it thinks it has found home
		case ROBOT_STATE_FindLineToHome:
			printf("FindLineToHome");
			
			if(line_width > 400 && line_width < 600)
			{
				printf("\nGOt It!\n");
				RobotState = ROBOT_STATE_DrivePastHome;
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.DoubleBeep();
				#endif
			}
			
			if(lineloc != 0xFFFF && line_width < 500 && line_width > 0  && lineloc < 300 && lineloc > -300)
			{
				//havent found goal yet - follow line
				#ifdef USE_ROBOT_SERIAL_PORT
				robot.DriveMotorDirect(ROBOT_SPEED_BASE-lineloc/CORRECTION_DIV,ROBOT_SPEED_BASE+lineloc/CORRECTION_DIV);
				#endif
			}else{
				//if lost - turn in place
				if(lineloc_last < 0)
				{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(ROBOT_SPEED_BASE,-ROBOT_SPEED_BASE);
					#endif
				}else{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(-ROBOT_SPEED_BASE,ROBOT_SPEED_BASE);
					#endif
				}
			}
			break;

		//FollowLineToHome - not used. summed up in previous state
		case ROBOT_STATE_FollowLineToHome:
			printf("FollowLineToHome");
			break;

		//DrivePastGoal - Automatic behavior to situate robot over home and make a 180 turn
		//When finished, robot withh go back to WaitForIDScan state.
		case ROBOT_STATE_DrivePastHome:
			printf("DrivePastHome");
			//Perform 'move forward' maneuver
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(ROBOT_SPEED_BASE,ROBOT_SPEED_BASE);
			#endif
			ret = waitKey(4300);
			//If user says this is not a goal - keep chuggin.
			if(ret == 'n') {RobotState = ROBOT_STATE_FindLineToHome; break;}
			//Perform 'turn 180' maneuver
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(-ROBOT_SPEED_BASE*2,ROBOT_SPEED_BASE*2);
			#endif
			ret = waitKey(3000);
			//Halt
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(0,0);
			#endif
			RobotState = ROBOT_STATE_WaitForIDScan;
			countdown = 0;
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.TripleBeep();
			#endif
			break;

		}
		printf("\n");
		//=========END ROBOT STATE MACHINE================

#endif

		//Write Data to logfile
		#ifdef MAKE_LOG_FILE
		sprintf(DataBuffer,"\tExiting State:  %c\n",(char)RobotState);
		dwBytesToWrite = (DWORD)strlen(DataBuffer);
		bErrorFlag = WriteFile( logfile, DataBuffer, dwBytesToWrite, &dwBytesWritten, NULL); 
		sprintf(DataBuffer,"\tLine: %04d | LineLast: %04d | LineWidth : %04d\n",lineloc,lineloc_last,line_width);
		dwBytesToWrite = (DWORD)strlen(DataBuffer);
		bErrorFlag = WriteFile( logfile, DataBuffer, dwBytesToWrite, &dwBytesWritten, NULL);   
		#endif

		//If escape key is pressed - stop robot and reset state machine
		if(ret == 27 || ret == 32) 
		{
			RobotState = ROBOT_STATE_Init;
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(0,0);
			robot.QuickBeep();
			#endif
		}
		//If q is pressed - quit (unsafely?)
		if(ret == 'q')
			break;

		//Draw to Window and look for keypress
		imshow("floatframe",colorfilt_linedet_frame);
		imshow("frame",origframe);
		ret = waitKey(1);


	}

	//Stop Robot
	printf("\nStopping Motors\n");
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.DriveMotorDirect(0,0);
	#endif

	//Close logfile
	#ifdef MAKE_LOG_FILE
	CloseHandle(logfile);
	#endif

	//Release Capture device
    cvReleaseCapture(&cap);

	//End of Line
    return 0;
}


/*#ifndef RUN_STATE_MACHINE

		//Apply Filters
		camframecopy.copyTo(origframe);											//get a new frame to work with
		GaussianBlur(origframe,blurframe,cv::Size(11,11),2.3,0);				//blur the frame
		filter_Color(blurframe,colorfilt_linedet_frame, colorMode, threshVal);	//filter frame for color
		//filter_Color(blurframe,blackfilt_linedet_frame, COLOR_BLACK, 0.25);	//filter frame for black
		Mat ylarg, ylarg2;
		cv::cvtColor(blurframe, ylarg, CV_BGR2HSV);
		inRange(ylarg, Scalar(21, 150, 100), Scalar(29, 255, 255), ylarg2);
		//imshow("clarg",clarg2);
		//blurframe.copyTo(colorfilt_linedet_frame,clarg2);
		cv::cvtColor(ylarg2, ylwfilt_linedet_frame, CV_GRAY2BGR);
		//Black Filter
		//Mat blarg,blarg2,goblinman;
		//cv::cvtColor(blurframe, blarg, CV_BGR2HSV);
		//inRange(blarg, Scalar(0, 0, 0), Scalar(180, 255, 10), blarg2);
		//cv::cvtColor(blarg2, blackfilt_linedet_frame, CV_GRAY2BGR);


		//Detect line location and width
		if(lineloc != 0xFFFF && line_width < 500) lineloc_last = lineloc;
		lineloc = LineFollow_getDeltaLineLoc(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8);
		line_width = LineFollow_getLineWidth(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) ;
		blacklineloc = LineFollow_getDeltaLineLoc(blackfilt_linedet_frame, blackfilt_linedet_frame.rows-20, blackfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8);
		blackline_width = LineFollow_getLineWidth(blackfilt_linedet_frame, blackfilt_linedet_frame.rows-20, blackfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) ;
		Draw_LineFollow_LineLoc(colorfilt_linedet_frame,colorfilt_linedet_frame,colorfilt_linedet_frame.rows-20, LINE_DELTA_THRESH_U8);		
		
		//Drive Robot
		if(lineloc != 0xFFFF && line_width < 500 )
			{
				if(last_was_lost)
				{
					Sleep(100);
					last_was_lost = 0;
				}else{
					last_was_lost = 0;
					//havent found goal yet - follow line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(ROBOT_SPEED_BASE-lineloc/CORRECTION_DIV,ROBOT_SPEED_BASE+lineloc/CORRECTION_DIV);
					#endif
				}
			} 
			//else if( blackline_width > 30 && blackline_width < 400)
			//{
			//	//found goal
			//	#ifdef USE_ROBOT_SERIAL_PORT
			//	robot.DriveMotorDirect(0,0);
			//	#endif
			//	RobotState = ROBOT_STATE_WaitForReturnCommand;
			//}
			else{
				last_was_lost = 1;
				if(lineloc_last < 0)
				{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(-ROBOT_SPEED_BASE,-ROBOT_SPEED_BASE);
					#endif
				}else{
					//find line
					#ifdef USE_ROBOT_SERIAL_PORT
					robot.DriveMotorDirect(-ROBOT_SPEED_BASE,-ROBOT_SPEED_BASE);
					#endif
				}
			}

		//======Debug Code======
		//printf("solid color green: %f\n\n",getIsFullScreenColor(origframe,COLOR_GREEN,0.5));
		//printf("line width: %d\n\n",LineFollow_getLineWidth(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, LINE_DELTA_THRESH_U8) );
		//printf("%d: ",lineloc);
		//(lineloc>0)?printf("Go Right\n") : printf("Go Left\n");


		
		//Display Screens
		#ifdef SHOW_SCREENS
		imshow("frame",camframecopy);
		#endif
		imshow("floatframe",colorfilt_linedet_frame);

		//End of this loop - what to do next?
		ret = waitKey(1);											//Nothing, keep going
		if( ret == 32 || ret == 13 )break;								// ENTER or SPACE key - end program
		if( ret == 98 ) {colorMode = COLOR_BLUE; threshVal = 0.39;}		// 'b' key - follow blue lines
		if( ret == 103 ) {colorMode = COLOR_GREEN; threshVal = 0.5;}	// 'g' key - follow green lines
		if( ret == 114 ) {colorMode = COLOR_RED; threshVal=0.67;}		// 'r' key - follow red lines
		if( ret == 107 ) {colorMode = COLOR_BLACK; threshVal=0.10;}		// 'k' key - follow black lines

#endif 
*/