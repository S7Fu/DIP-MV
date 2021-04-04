#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
//#define READIMAGE_ONLY
//#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
//#endif

using namespace cv;
using namespace std;

//直方图绘制函数，参数vector<int> nums 是灰度图片256级灰度的像素个数
void drawHist(vector<int> nums)
{
    //利用Mat类新建一个600×800的黑色画布，在该画布上绘制灰度直方图。
	Mat hist = Mat::zeros(600, 800, CV_8UC3);
    //灰度图片256级灰度的像素个数中最大的数量
	int Max = *max_element(nums.begin(), nums.end());
	//auto Max = max_element(nums.begin(), nums.end()); //max迭代器类型,最大数目
    putText(hist, "Histogram", Point(150, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);
    //x轴
    line(hist, o, x, Scalar(255, 255, 255), 2, 8, 0);
    //y轴
    line(hist, o, y, Scalar(255, 255, 255), 2, 8, 0);

    //********绘制灰度曲线***********//
    Point pts[256];
    //生成坐标点
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums[i] * (300.0 / (Max))); //归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }
    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(0, 255, 0), 2);
    }
    //显示图像
    imshow("Hist", hist);
}

//统计每个灰度下的像素个数
void calHist(Mat gray)
{
	vector<int> nums(256);	
	//统计每个灰度下的像素个数
    for (int i = 0; i < gray.rows; i++)
    {
        for (int j = 0; j < gray.cols; j++)
        {
            nums[gray.at<uchar>(i, j)]++;
        }
    }
	//绘出直方图
    drawHist(nums);
}

Mat equalize(Mat src)
{
    Mat equal = src.clone();
    //统计每个灰度下的像素个数
    int nums[256] = { 0 };  //每个灰度下的像素个数
    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            nums[src.at<uchar>(i, j)]++;
        }
    }
    //统计灰度频率
    int pixels = src.cols * src.rows;  //像素总数
    double gray_feq[256] = { 0 };  //灰度分布频率
    for (int i = 0; i < 256; i++)
    {
        gray_feq[i] = ((double)nums[i] / pixels);
    }
    //计算累计密度
    double gray_dens[256] = { 0 };  //累计密度
    gray_dens[0] = gray_feq[0];
    for (int i = 1; i < 256; i++)   
    {
        gray_dens[i] = gray_dens[i-1] +gray_feq[i];
    }
    //重新计算均衡化后的灰度值
    int gray_eq[256] = { 0 };  //均衡化后的灰度值
    for (int i = 0; i < 256; i++)
    {
        gray_eq[i] = (uchar)(255 * gray_dens[i] + 0.5);
    }
    //直方图均衡化,更新原图每个点的像素值
    for (int i = 0; i < equal.rows; i++)
    {
        uchar* p = equal.ptr<uchar>(i);
        for (int j = 0; j < equal.cols; j++)
        {
            p[j] = gray_eq[p[j]];
        }
    }
	return equal;
}

int main(int argc, char **argv)
{
	ROS_WARN("*****START*****");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
	ros::NodeHandle n;

	//Before the use of camera, you can test ur program with images first: imread()
	VideoCapture capture;
	capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	waitKey(100);
	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}

#ifndef READIMAGE_ONLY
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif
	Mat src_frame;
	Mat gray;

	while (ros::ok())
	{
		capture.read(src_frame);
		imshow("src", src_frame);
		// 此处为实验部分，请自行增加直方图均衡化的代码
		//转化为灰度图
		cvtColor(src_frame,gray, COLOR_RGB2GRAY);
		imshow("GRAY",gray);
		//统计每个灰度下的像素个数并绘出直方图
		//calHist(gray);
		//直方图均衡化并绘出直方图
		Mat equal = equalize(gray);
		imshow("equal",equal);
		calHist(equal);

//#ifndef READIMAGE_ONLY
		//以下代码可设置机器人的速度值，从而控制机器人运动
		geometry_msgs::Twist cmd_red;
		cmd_red.linear.x = 0;
		cmd_red.linear.y = 0;
		cmd_red.linear.z = 0;
		cmd_red.angular.x = 0;
		cmd_red.angular.y = 0;
		cmd_red.angular.z = 0.2;
		pub.publish(cmd_red);
//#endif
		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}