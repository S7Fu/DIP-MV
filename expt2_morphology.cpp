#include <stdlib.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0

using namespace cv;
using namespace std;

//////////////////////滤波//////////////////
// 空域高斯滤波器函数
Mat Gaussian(Mat input, int n, double sigma){
    // 申请一个二维数组，存放生成的高斯模板矩阵
	double **templateM = new double*[n];
	for(int i = 0; i < n; i++)
		templateM[i] = new double[n];
	int center = n/2;   //以模板的中心为原点
	double x2, y2;
	double sum = 0;
	for(int i = 0; i < n; i++)
	{
		x2 = pow(i-center, 2);
		for(int j = 0; j < n; j++)
		{
			y2 = pow(j-center, 2);
			double h =exp(-(x2+y2)/(2*sigma*sigma));
			sum += h;
			templateM[i][j] = h;
		}
	}
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
			templateM[i][j] /= sum;
	}
	//将模板应用到图像中
	Mat output = input.clone();
	for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
			double sum = 0;
			for(int x = -center; x <= center; x++)
			{
				for(int y = -center; y <= center; y++)
					sum += templateM[center+x][center+y] * input.at<uchar>(i + x, j + y);
			}
			output.at<uchar>(i, j) = static_cast<uchar>(sum);
		}
	}
	for(int i = 0; i < n; i++)
		delete[] templateM[i];
	delete[] templateM;
	return output;
}

//////////////////////形态学//////////////////
// 膨胀函数
Mat Dilate(Mat input, int n){
	double **templateM = new double*[n];
	for(int i = 0; i < n; i++)
		templateM[i] = new double[n];
	int center = n/2;   //以模板的中心为原点
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
			templateM[i][j] = 0;
	}
	//将模板应用到图像中
	Mat output = input.clone();
	for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
			int sum = 0;
			for(int x = -center; x <= center; x++)
			{
				for(int y = -center; y <= center; y++)
				{
					if(templateM[center+x][center+y] == input.at<uchar>(i + x, j + y))
						sum += 1;
				}
			}
			if(sum != 0)
				output.at<uchar>(i, j) = 0;
			else
				output.at<uchar>(i, j) = 255;
		}
	}
	for(int i = 0; i < n; i++)
		delete[] templateM[i];
	delete[] templateM;
	return output;
}

// 腐蚀函数
Mat Erode(Mat input, int n){
	double **templateM = new double*[n];
	for(int i = 0; i < n; i++)
		templateM[i] = new double[n];
	int center = n/2;   //以模板的中心为原点
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
			templateM[i][j] = 0;
	}
	//将模板应用到图像中
	Mat output = input.clone();
	for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
			int sum = 0;
			for(int x = -center; x <= center; x++)
			{
				for(int y = -center; y <= center; y++)
				{
					if(templateM[center+x][center+y] == input.at<uchar>(i + x, j + y))
						sum += 1;
				}
			}
			if(sum == 9)
				output.at<uchar>(i, j) = 0;
			else
				output.at<uchar>(i, j) = 255;
		}
	}
	for(int i = 0; i < n; i++)
		delete[] templateM[i];
	delete[] templateM;
	return output;
}

int main(int argc, char **argv)
{
    //VideoCapture capture;
    //capture.open(1); // 1 为打开 zed 相机, 0 为打开笔记本摄像头
    
    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
    ros::NodeHandle n;
    
    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);// 定义速度发布器
    
    //if (!capture.isOpened())
    //{
    //    printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
    //    return 0;
    //}
    waitKey(1000);
    //Mat frame;//当前帧图片
    Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/lena.jpeg");
    Mat gray, threshold_;
	//int nFrames = 0;//图片帧数
    //int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    //int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
    
    while (ros::ok())
    {
        //capture.read(frame);
        if(frame.empty())
        {
            break;
        }
		imshow("src", frame);
        //Mat frIn = frame.clone();//使用笔记本摄像头
        //Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
		
		//转化为灰度图
		cvtColor(frame, gray, COLOR_RGB2GRAY);
		imshow("GRAY",gray);
        
		// 空域滤波函数
        Mat smooth = Gaussian(gray, 5, 0.8);
		imshow("smooth", smooth);
    
		threshold(gray, threshold_, 100, 255, THRESH_BINARY);
		imshow("threshold", threshold_);
        // 膨胀函数
        Mat dilate = Dilate(threshold_, 5);
		imshow("dilate", dilate);
    
        // 腐蚀函数
		Mat erode = Erode(threshold_, 5);
		imshow("erode", erode);
    
        //imshow("1",frIn);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}