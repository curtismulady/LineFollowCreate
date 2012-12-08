
#include "stdafx.h"
#include "common.h"



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

float getIsFullScreenColor(Mat& src, COLORNAME_t color, float pct_thresh)
{
	//Arguments
	//	color: COLOR_RED, COLOR_GREEN, COLOR_BLUE, or COLOR_BLACK for color filter selection
	//	pct_thresh: [0.0 to 1.0] percent threshold of color dominance (in a r:g:b ratio) to "accept" as "of the right color". Typ~=[0.4 to 0.5]
	//		-for RED, GRN, and BLU: actual pixel ratio must be greater than pct_thresh
	//		-for BLK: all color channels must be less than pct_thresh [ 0.0 to 1.0 ]

	Mat pre_dst;
	unsigned long match_count = 0;
	//Convert image frame to float
	src.convertTo(pre_dst,CV_32FC3,1./255);
	for (int i = 0; i < pre_dst.rows; i++)
	{
		for (int j = 0; j < pre_dst.cols; j++)
		{
			if(color == COLOR_GREEN || color == COLOR_RED || color == COLOR_BLUE)
			{
				//For each pixel: compute relative levels
				float color_levels[3];
				float total = pre_dst.at<Vec3f>(i,j)[0] + pre_dst.at<Vec3f>(i,j)[1] + pre_dst.at<Vec3f>(i,j)[2];
				for(int k=0; k<3; k++) color_levels[k] = pre_dst.at<Vec3f>(i,j)[k]/total;
				//check to see if color pixel thresh=good 
				if(color_levels[(int)color]>pct_thresh )
				{
					match_count+=1.;
				}else{
					//do nothing
				}
			}
		}
	}
	return((float)match_count/(float)(pre_dst.rows*pre_dst.cols));
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

bool LineFollow_isFullLine(Mat& src, int row_to_watch,int centerline_col, int delta_thresh)
{
	//see if all pixels on line are of right color
	Mat grey;
	bool success = true;
	cv::cvtColor(src,grey,CV_BGR2GRAY);
	int pos_dv_x_loc = 0, pos_dv_val = 0;
	int neg_dv_x_loc = 0, neg_dv_val= 0;
	for(int i=1; i<grey.cols-1; i++){
		if(grey.data[row_to_watch*grey.step+i]==0)
			success = false;
	}
	//Return width of line
	return(success);
}

int LineFollow_getLineWidth(Mat& src, int row_to_watch,int centerline_col, int delta_thresh)
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
	//Return width of line
	return(abs(pos_dv_x_loc - neg_dv_x_loc));
}

void filter_Color(Mat& src, Mat& dst, COLORNAME_t color, float pct_thresh)
{
	//Arguments
	//	color: COLOR_RED, COLOR_GREEN, COLOR_BLUE, or COLOR_BLACK for color filter selection
	//	pct_thresh: [0.0 to 1.0] percent threshold of color dominance (in a r:g:b ratio) to "accept" as "of the right color". Typ~=[0.4 to 0.5]
	//		-for RED, GRN, and BLU: actual pixel ratio must be greater than pct_thresh
	//		-for BLK: all color channels must be less than pct_thresh [ 0.0 to 1.0 ]

	Mat pre_dst;
	//Convert image frame to float
	src.convertTo(pre_dst,CV_32FC3,1./255);
	for (int i = 0; i < pre_dst.rows; i++)
	{
		for (int j = 0; j < pre_dst.cols; j++)
		{
			if(color == COLOR_GREEN || color == COLOR_RED || color == COLOR_BLUE)
			{
				//For each pixel: compute relative levels
				float color_levels[3];
				float total = pre_dst.at<Vec3f>(i,j)[0] + pre_dst.at<Vec3f>(i,j)[1] + pre_dst.at<Vec3f>(i,j)[2];
				for(int k=0; k<3; k++) color_levels[k] = pre_dst.at<Vec3f>(i,j)[k]/total;
				//keep color pixel if thresh=good 
				if(color_levels[(int)color]>pct_thresh )
				{
					pre_dst.at<Vec3f>(i,j)[0] = pre_dst.at<Vec3f>(i,j)[(int)color];
					pre_dst.at<Vec3f>(i,j)[1] = pre_dst.at<Vec3f>(i,j)[(int)color];
					pre_dst.at<Vec3f>(i,j)[2] = pre_dst.at<Vec3f>(i,j)[(int)color];
				}else{
					pre_dst.at<Vec3f>(i,j)[0] = 0.;
					pre_dst.at<Vec3f>(i,j)[1] = 0.;
					pre_dst.at<Vec3f>(i,j)[2] = 0.;
				}
			}else if(color == COLOR_BLACK)
			{
				if(pre_dst.at<Vec3f>(i,j)[0] < pct_thresh && pre_dst.at<Vec3f>(i,j)[1] < pct_thresh && pre_dst.at<Vec3f>(i,j)[2] < pct_thresh)
				{
					pre_dst.at<Vec3f>(i,j)[0] = 1.-pre_dst.at<Vec3f>(i,j)[0];
					pre_dst.at<Vec3f>(i,j)[1] = 1.-pre_dst.at<Vec3f>(i,j)[1];
					pre_dst.at<Vec3f>(i,j)[2] = 1.-pre_dst.at<Vec3f>(i,j)[2];
				}else{
					pre_dst.at<Vec3f>(i,j)[0] = 0.;
					pre_dst.at<Vec3f>(i,j)[1] = 0.;
					pre_dst.at<Vec3f>(i,j)[2] = 0.;
				}
			}
		}
	}
	//convert output frame to unsigned char - 3 channel
	pre_dst.convertTo(dst,CV_8UC3,255);

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
