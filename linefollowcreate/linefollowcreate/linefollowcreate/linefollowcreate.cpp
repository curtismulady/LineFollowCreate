// opencv_test_pgm.cpp : main project file.

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define __PI__ 3.14159 

using namespace System;
using namespace cv;
using namespace std;

void DrawRhoThetaLine(Mat& dst, float rho, float theta, Scalar color, int width);
void InvertColorMat(Mat& dst);
void Draw_LineFollow_LineLoc(Mat& src, Mat& dst, int row_to_watch);
int LineFollow_getDeltaLineLoc(Mat& src, int row_to_watch,int centerline_col);

CvCapture* cap = NULL;
Mat camframe,camframecopy,invframe,blurframe,cannyframe,houghframe,greyframe;
IplImage* iplimg;

vector<cv::Vec2f> lines;
vector<cv::Vec2f>::iterator it;


int main(array<System::String ^> ^args)
{
    Console::WriteLine(L"Starting...");


	cap = cvCaptureFromCAM(0);
	if(!cap) printf ("No camera detected.");

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
		camframecopy.copyTo(invframe);
		InvertColorMat(invframe);
		GaussianBlur(invframe,blurframe,cv::Size(11,11),2.3,0);
		// draw line loc stuff
		Draw_LineFollow_LineLoc(blurframe,camframecopy,blurframe.rows-20);
		// do smart stuff
		int lineloc = LineFollow_getDeltaLineLoc(blurframe, blurframe.rows-20, blurframe.cols/2);
		printf("%d: ",lineloc);
		(lineloc>0)?printf("Go Right\n") : printf("Go Left\n");




		


		imshow("frame",camframecopy);


		waitKey(1);




	}
	




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

int LineFollow_getDeltaLineLoc(Mat& src, int row_to_watch,int centerline_col)
{
	//find big pos dv and big neg dv on specd row
	Mat grey;
	cv::cvtColor(src,grey,CV_BGR2GRAY);
	int pos_dv_x_loc = 0, pos_dv_val = 0;
	int neg_dv_x_loc = 0, neg_dv_val= 0;
	for(int i=1; i<grey.cols-1; i++){
		int temp_dv = grey.data[row_to_watch*grey.step+i] - grey.data[row_to_watch*grey.step+i+1];
		if(temp_dv > pos_dv_val){
			pos_dv_x_loc = i;
			pos_dv_val = temp_dv;
		}
		if(temp_dv < neg_dv_val){
			neg_dv_x_loc = i;
			neg_dv_val = temp_dv;
		}	
	}
	int line_loc = (pos_dv_x_loc + neg_dv_x_loc) / 2;
	return(line_loc - centerline_col);
}

void Draw_LineFollow_LineLoc(Mat& src, Mat& dst, int row_to_watch)
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