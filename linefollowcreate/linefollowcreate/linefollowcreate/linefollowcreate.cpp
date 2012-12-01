// opencv_test_pgm.cpp : main project file.

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"

//#define USE_ROBOT_SERIAL_PORT

#define __PI__ 3.14159 
#define COMNUM 1
#define CORRECTION_DIV 7

using namespace System;
using namespace cv;
using namespace std;

typedef enum {COLOR_RED=2,COLOR_GREEN=1,COLOR_BLUE=0} COLORNAME_t;

void DrawRhoThetaLine(Mat& dst, float rho, float theta, Scalar color, int width);
void InvertColorMat(Mat& dst);
void Draw_LineFollow_LineLoc(Mat& src, Mat& dst, int row_to_watch, int delta_thresh);
int LineFollow_getDeltaLineLoc(Mat& src, int row_to_watch,int centerline_col, int delta_thresh);
void filter_Color(Mat& src, Mat& dst, COLORNAME_t color, float pct_thresh);
void Draw_LineFollow_LineLoc_floatMat(Mat& src, Mat& dst, int row_to_watch);

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

		// invert and blur the image
		camframecopy.copyTo(origframe);
		GaussianBlur(camframecopy,camframecopy,cv::Size(11,11),2.3,0);
		// draw line loc stuff
		// do smart stuff
		
		
		


		filter_Color(origframe,colorfiltframe, colorMode, threshVal); //47
		//filter_Color(origframe,colorfiltframe, COLOR_RED, 0.67);
		//filter_Color(origframe,colorfiltframe, COLOR_GREEN, 0.5);
		colorfiltframe.convertTo(colorfilt_linedet_frame,CV_8UC3,255);
		int lineloc = LineFollow_getDeltaLineLoc(colorfilt_linedet_frame, colorfilt_linedet_frame.rows-20, colorfilt_linedet_frame.cols/2, 40);
		Draw_LineFollow_LineLoc(colorfilt_linedet_frame,colorfilt_linedet_frame,colorfilt_linedet_frame.rows-20, 40);		
		printf("%d: ",lineloc);
		(lineloc>0)?printf("Go Right\n") : printf("Go Left\n");

		if(lineloc != 0xFFFF)
		{
			//follow line
			printf("R=%d: L=%d\n",50-lineloc,50+lineloc);
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(80-lineloc/CORRECTION_DIV,80+lineloc/CORRECTION_DIV);
			#endif
		}else{
			//find line
			printf("Oh,crap! Lost the line.\n");
			#ifdef USE_ROBOT_SERIAL_PORT
			robot.DriveMotorDirect(80,-80);
			#endif

		}



		


		imshow("frame",camframecopy);
		imshow("floatframe",colorfilt_linedet_frame);

		int ret = waitKey(1);
		if( ret == 32 || ret == 13 )break;
		if( ret == 98 ) {colorMode = COLOR_BLUE; threshVal = 0.47;}
		if( ret == 103 ) {colorMode = COLOR_GREEN; threshVal = 0.5;}
		if( ret == 114 ) {colorMode = COLOR_RED; threshVal=0.67;}




	}
	
	printf("\nStopping Motors\n");
	#ifdef USE_ROBOT_SERIAL_PORT
	robot.DriveMotorDirect(0,0);
	#endif



    return 0;
}


void DrawRhoThetaLine(Mat& dst, float rho, float theta, Scalar color, int width)
{
	if(theta < __PI__/4.0 || theta > 3.0*__PI__/4.0)
	{
		cv::Point pt1((int)(rho/cos(theta)),0);
		cv::Point pt2((int)((rho-dst.rows*sin(theta))/cos(theta)),dst.rows);
		cv::line(dst,pt1,pt2,color,width);
	}else{
		cv::Point pt1(0,(int)(rho/sin(theta)));
		cv::Point pt2(dst.cols,(int)((rho-dst.cols*cos(theta))/sin(theta)));
		cv::line(dst,pt1,pt2,color,width);
	}
}

int LineFollow_getDeltaLineLoc(Mat& src, int row_to_watch,int centerline_col, int delta_thresh)
{
	//find big pos dv and big neg dv on specd row
	Mat grey;
	cv::cvtColor(src,grey,CV_BGR2GRAY);
	int pos_dv_x_loc = 0, pos_dv_val = 0;
	int neg_dv_x_loc = 0, neg_dv_val= 0;
	for(int i=1; i<grey.cols-1; i++){
		int temp_dv = grey.data[row_to_watch*grey.step+i] - grey.data[row_to_watch*grey.step+i+1];
		if(temp_dv > pos_dv_val && temp_dv > delta_thresh){
			pos_dv_x_loc = i;
			pos_dv_val = temp_dv;
		}
		if(temp_dv < neg_dv_val && temp_dv < -1*delta_thresh){
			neg_dv_x_loc = i;
			neg_dv_val = temp_dv;
		}	
	}
	int line_loc = (pos_dv_x_loc + neg_dv_x_loc) / 2;
	if(neg_dv_x_loc==0 && pos_dv_x_loc==0)
	{
		return(0xFFFF);
	}else{
		return(line_loc - centerline_col);
	}
}

void filter_Color(Mat& src, Mat& dst, COLORNAME_t color, float pct_thresh)
{
	//Arguments
	//	color: COLOR_RED, COLOR_GREEN, or COLOR_BLUE for color filter selection
	//	pct_thresh: percent threshold of color dominance (in a r:g:b ratio) to "accept" as "of the right color". Typ~=[0.4 to 0.5]
	src.convertTo(dst,CV_32FC3,1./255);
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			//For each pixel: compute relative levels
			float color_levels[3];
			float total = dst.at<Vec3f>(i,j)[0] + dst.at<Vec3f>(i,j)[1] + dst.at<Vec3f>(i,j)[2];
			for(int k=0; k<3; k++) color_levels[k] = dst.at<Vec3f>(i,j)[k]/total;
			//keep color pixel if thresh=good 
			if(color_levels[(int)color]>pct_thresh )
			{
				dst.at<Vec3f>(i,j)[0] = dst.at<Vec3f>(i,j)[(int)color];
				dst.at<Vec3f>(i,j)[1] = dst.at<Vec3f>(i,j)[(int)color];
				dst.at<Vec3f>(i,j)[2] = dst.at<Vec3f>(i,j)[(int)color];
			}else{
				dst.at<Vec3f>(i,j)[0] = 0.;
				dst.at<Vec3f>(i,j)[1] = 0.;
				dst.at<Vec3f>(i,j)[2] = 0.;
			}
		}
	}
}

void Draw_LineFollow_LineLoc(Mat& src, Mat& dst, int row_to_watch, int delta_thresh)
{
	//find big pos dv and big neg dv on specd row
	Mat grey;
	cv::cvtColor(src,grey,CV_BGR2GRAY);
	cv::line(dst,cv::Point(0,row_to_watch-2),cv::Point(grey.cols,row_to_watch-2),cv::Scalar(0,0,255));
	cv::line(dst,cv::Point(0,row_to_watch+2),cv::Point(grey.cols,row_to_watch+2),cv::Scalar(0,0,255));
		
	int pos_dv_x_loc = 0, pos_dv_val = 0;
	int neg_dv_x_loc = 0, neg_dv_val= 0;
	for(int i=1; i<grey.cols-1; i++)
	{
		int temp_dv = grey.data[row_to_watch*grey.step+i] - grey.data[row_to_watch*grey.step+i+1];
		if(temp_dv > pos_dv_val && temp_dv > delta_thresh)
		{
			pos_dv_x_loc = i;
			pos_dv_val = temp_dv;
		}
		if(temp_dv < neg_dv_val && temp_dv < -1*delta_thresh)
		{
			neg_dv_x_loc = i;
			neg_dv_val = temp_dv;
		}

		grey.data[(row_to_watch-2)*grey.step+i] = 0;
		grey.data[(row_to_watch+2)*grey.step+i] = 0;
			
	}
	if(neg_dv_x_loc==0 && pos_dv_x_loc==0)
	{

	}else{
		cv::rectangle(dst,cv::Point(pos_dv_x_loc,row_to_watch-3),cv::Point(neg_dv_x_loc,row_to_watch+3),cv::Scalar(0,255,0));
		cv::rectangle(dst,cv::Point((pos_dv_x_loc+neg_dv_x_loc)/2-1,row_to_watch-1),cv::Point((pos_dv_x_loc+neg_dv_x_loc)/2+1,row_to_watch+1),cv::Scalar(0,255,0));
	}
	
}

void Draw_LineFollow_LineLoc_floatMat(Mat& src, Mat& dst, int row_to_watch)
{
	//find big pos dv and big neg dv on specd row
	Mat grey;
	cv::cvtColor(src,grey,CV_BGR2GRAY);
	cv::line(dst,cv::Point(0,row_to_watch-2),cv::Point(grey.cols,row_to_watch-2),cv::Scalar(0,0,255));
	cv::line(dst,cv::Point(0,row_to_watch+2),cv::Point(grey.cols,row_to_watch+2),cv::Scalar(0,0,255));
		
	/*int pos_dv_x_loc = 0, pos_dv_val = 0;
	int neg_dv_x_loc = 0, neg_dv_val= 0;
	for(int i=1; i<grey.cols-1; i++)
	{
		int temp_dv = grey.data[row_to_watch*grey.step+i] - grey.data[row_to_watch*grey.step+i+1];
		if(temp_dv > pos_dv_val)
		{
			pos_dv_x_loc = i;
			pos_dv_val = temp_dv;
		}
		if(temp_dv < neg_dv_val)
		{
			neg_dv_x_loc = i;
			neg_dv_val = temp_dv;
		}

		grey.data[(row_to_watch-2)*grey.step+i] = 0;
		grey.data[(row_to_watch+2)*grey.step+i] = 0;
			
	}
	cv::rectangle(dst,cv::Point(pos_dv_x_loc,row_to_watch-3),cv::Point(neg_dv_x_loc,row_to_watch+3),cv::Scalar(0,255,0));
	cv::rectangle(dst,cv::Point((pos_dv_x_loc+neg_dv_x_loc)/2-1,row_to_watch-1),cv::Point((pos_dv_x_loc+neg_dv_x_loc)/2+1,row_to_watch+1),cv::Scalar(0,255,0));
	*/
}

void InvertColorMat(Mat& dst)
{
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			dst.data[dst.step * i + j * dst.channels() + 0] = 255 - dst.data[dst.step * i + j * dst.channels() + 0];
			dst.data[dst.step * i + j * dst.channels() + 1] = 255 - dst.data[dst.step * i + j * dst.channels() + 1];
			dst.data[dst.step * i + j * dst.channels() + 2] = 255 - dst.data[dst.step * i + j * dst.channels() + 2];

		}
	}
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