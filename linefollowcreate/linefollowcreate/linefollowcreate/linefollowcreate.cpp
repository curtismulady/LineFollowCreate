// opencv_test_pgm.cpp : main project file.

#include "stdafx.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace System;
using namespace cv;
using namespace std;

CvCapture* cap = NULL;
Mat camframe,camframecopy,blurframe,cannyframe;
IplImage* iplimg;


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

		imshow("canny",cannyframe);
		imshow("frame",camframe);



		waitKey(10);

	}
	




    return 0;
}
