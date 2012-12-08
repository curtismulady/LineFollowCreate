
#ifndef COMMON_H
#define COMMON_H

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"

#define __PI__ 3.14159 

using namespace System;
using namespace cv;
using namespace std;

typedef enum {COLOR_RED=2,COLOR_GREEN=1,COLOR_BLUE=0,COLOR_BLACK=3} COLORNAME_t;



void DrawRhoThetaLine(Mat& dst, float rho, float theta, Scalar color, int width);
void InvertColorMat(Mat& dst);

void Draw_LineFollow_LineLoc(Mat& src, Mat& dst, int row_to_watch, int delta_thresh);
int LineFollow_getDeltaLineLoc(Mat& src, int row_to_watch,int centerline_col, int delta_thresh);
int LineFollow_getLineWidth(Mat& src, int row_to_watch,int centerline_col, int delta_thresh);
bool LineFollow_isFullLine(Mat& src, int row_to_watch,int centerline_col, int delta_thresh);

void filter_Color(Mat& src, Mat& dst, COLORNAME_t color, float pct_thresh);
float getIsFullScreenColor(Mat& src, COLORNAME_t color, float pct_thresh);


#endif