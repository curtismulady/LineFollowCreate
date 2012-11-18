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

CvCapture* cap = NULL;
Mat camframe,camframecopy,blurframe,cannyframe,houghframe;
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

		GaussianBlur(camframe,blurframe,cv::Size(11,11),2,0);
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
		imshow("frame",camframe);

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