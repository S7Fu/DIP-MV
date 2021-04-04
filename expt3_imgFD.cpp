#include <iostream>
#include <stdlib.h>
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

//////////////////////边缘检测//////////////////
//边缘检测函数
Mat EdgeDetector(Mat input){
    //用Sobel算子计算梯度
    int center = 3/2;
    int SobelX[3][3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
    int SobelY[3][3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
    double Direction[input.rows][input.cols] = {0};
    Mat Sobel = input.clone();
    //Mat Sobel1 = input.clone();
    //Mat Sobel2 = input.clone();
	for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
            double Gx = 0, Gy = 0, G = 0, Theta = 0;
			for(int x = -center; x <= center; x++)
			{
				for(int y = -center; y <= center; y++)
                {
                    Gx += SobelX[center+x][center+y] * input.at<uchar>(i + x, j + y);
                    Gy += SobelY[center+x][center+y] * input.at<uchar>(i + x, j + y);
                }
			}
            if(Gx == 0)
                Gx = 0.0000000000001;
            Theta = atan(Gy/Gx)*180/3.14159268;
            Direction[i][j] = Theta;
            G = sqrt(Gx*Gx + Gy*Gy);
            if(G > 255)
                G = 255;
			Sobel.at<uchar>(i, j) = static_cast<uchar>(G);
            //Sobel1.at<uchar>(i, j) = static_cast<uchar>(abs(Gx));
            //Sobel2.at<uchar>(i, j) = static_cast<uchar>(abs(Gy));
		}
	}
    imshow("Sobel", Sobel);
    //imshow("Sobel1", Sobel1);
    //imshow("Sobel2", Sobel2);
    
    //非极大值抑制
    Mat output = Sobel.clone();
    for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
            double G0 = 0, G1 = 0, G2 = 0;
            G0 = Sobel.at<uchar>(i, j);
            if(Direction[i][j] > 22.5 && Direction[i][j] <= 67.5){
                G1 = Sobel.at<uchar>(i-1, j-1);
                G2 = Sobel.at<uchar>(i+1, j+1);
            }
            else if(Direction[i][j] > -22.5 && Direction[i][j] <= 22.5){
                G1 = Sobel.at<uchar>(i-1, j);
                G2 = Sobel.at<uchar>(i+1, j);
            }
            else if(Direction[i][j] > -67.5 && Direction[i][j] <= -22.5){
                G1 = Sobel.at<uchar>(i-1, j+1);
                G2 = Sobel.at<uchar>(i+1, j-1);
            }
            else{
                G1 = Sobel.at<uchar>(i, j-1);
                G2 = Sobel.at<uchar>(i, j+1);
            }
            if(G0 < G1 || G0 < G2)
                output.at<uchar>(i, j) = 0;
		}
	}
    //滞后阈值
    for(int i = center; i < output.rows-center; i++)
	{
		for(int j = center; j < output.cols-center; j++)
		{
            int th = output.at<uchar>(i, j);
            if(th > 100)
                output.at<uchar>(i, j) = 255;
            else if(th < 40)
                output.at<uchar>(i, j) = 0;
		}
	}
    for(int i = center; i < input.rows-center; i++)
	{
		for(int j = center; j < input.cols-center; j++)
		{
            //对于两阈值之间的像素
            if (output.at<uchar>(i, j) != 0 && output.at<uchar>(i, j) != 255)
             {
                for (int x = -center; x < center; x++) 
                {
                    for (int y = -center; y < center; y++)
                    {
                        //仅仅在连接到一个高于高阈值的像素时被保留
                        if (output.at<uchar>(i+x, j+y) == 255)
                            output.at<uchar>(i, j) = 255;
                        else
                            output.at<uchar>(i, j) = 0;
                    }
                }
            }
        }
    }
	return output;
}

//////////////////////霍夫线变换//////////////////
Mat Hough_Line(Mat input){
    Mat output;
    cvtColor(input, output, COLOR_GRAY2RGB);
    //参数空间初始化
    int Theta = 180;
    int R = sqrt(input.rows*input.rows + input.cols*input.cols);
    float Cos[Theta] = {0}, Sin[Theta] = {0};
    for(int i = 0; i <= Theta; i++){
        Cos[i] = cos(i*M_PI/180);
        Sin[i] = sin(i*M_PI/180);
    }
    //累加器A
    int **A = new int*[R];
    for(int i = 0; i < R; i++)
        A[i] = new int[Theta];
    for(int i = 0; i < R; i++)
        for(int j = 0; j < Theta; j++)
            A[i][j] = 0;
    //参数空间离散化
    for(int i = 1; i < input.rows-1; i++){
		for(int j = 1; j < input.cols-1; j++){
            if(input.at<uchar>(i, j) == 255){
                for(int k = 0; k < Theta; k++)
                {
                    int r = (int)(i*Cos[k] + j*Sin[k]);
                    if(r >= 0)
                    {
                        A[r][k] += 1;
                        //printf("r = %d, theta = %d. ", r, k);
                    }
                }
            }
		}
	}
    //找出累加次数大的
    for(int i = 0; i < R; i++){
        for(int j = 0; j < Theta; j++){
            if(A[i][j] > 120)
            {
                for(int x = 1; x < input.rows-1; x++)
                {
                    for(int y = 1; y < input.cols-1; y++)
                    {
                        if(input.at<uchar>(x, y) == 255)
                        {
                            if(i == (int)(x*Cos[j] + y*Sin[j]))
                            {
                                circle(output, Point(y, x), 0.5, Scalar(255,0,0), -1);
                            }
                        }
                    }
                }
            }
        }
    }
    for(int i = 0; i < R; i++)
        delete[] A[i];
    delete[] A;
    return output;
}

//////////////////////霍夫圆变换//////////////////
Mat Hough_Circle(Mat input){
    Mat output;
    cvtColor(input, output, COLOR_GRAY2RGB);
    //参数空间初始化
    int Theta = 180;
    int R = 0;
    if(input.rows/2 > input.cols/2)
        R = (int)input.cols/2;
    else
        R = (int)input.rows/2;
    float Cos[Theta] = {0}, Sin[Theta] = {0};
    for(int i = 0; i <= Theta; i++){
        Cos[i] = cos(i*M_PI/180);
        Sin[i] = sin(i*M_PI/180);
    }
    //累加器A
    int*** A;
    A = new int**[input.rows];
    for(int i = 0; i < input.rows; i++)
    {
        A[i] = new int*[input.cols];
        for(int j = 0; j < input.cols; j++)
            A[i][j] = new int[R];
    }
    for(int i = 0; i < input.rows; i++)
        for(int j = 0; j < input.cols; j++)
            for(int k = 0; k < R; k++)
                A[i][j][k] = 0;

    //参数空间离散化
    for(int i = 1; i < input.rows-1; i++){
		for(int j = 1; j < input.cols-1; j++){
            if(input.at<uchar>(i, j) == 255)   //遍历边缘点
            {
                for(int r = 50; r < R; r++){
                    for(int k = 0; k < Theta; k++)
                    {
                        int a = (int)(i - r*Cos[k]);
                        int b = (int)(j - r*Sin[k]);
                        if(a < input.rows && b < input.cols && a > 0 && b > 0)
                        {
                            A[a][b][r] += 1;
                        }   
                    }
                }
            }
		}
	}
    
    //找出累加次数大的
    for(int i = 0; i < input.rows; i++){
        for(int j = 0; j < input.cols; j++){
            for(int r = 50; r < R; r++){
                if(A[i][j][r] > 150){
                    circle(output, Point(j, i), r, Scalar(0, 0, 255), 1);
                }
            }
        }
    }
    
    for(int i = 0; i < input.rows; i++)
    {
        for(int j = 0; j < input.cols; j++)
            delete[] A[i][j];
        delete[] A[i];
    }
    delete[] A;
    return output;
}

int main(int argc, char **argv)
{
    //VideoCapture capture;
    //capture.open(0);//打开 zed 相机
    
    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
    ros::NodeHandle n;
        
    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器
    
    //if (!capture.isOpened())
    //{
    //    printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
    //    return 0;
    //}
    waitKey(1000);
    //Mat frame;//当前帧图片
    //Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/lena.jpeg");
    Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/line.png");
    //Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/circle.png");
    Mat gray;
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

        //Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        
        // 灰度图转换
        imshow("src", frame);
        cvtColor(frame, gray, COLOR_RGB2GRAY);
		imshow("GRAY",gray);
        
        // 边缘检测函数
        Mat smooth = Gaussian(gray, 3, 0.8);
		imshow("smooth", smooth);
        Mat canny = EdgeDetector(smooth);
        imshow("canny", canny);

        // 线检测
        Mat houghL = Hough_Line(canny);
        imshow("houghL", houghL);

        // 圆检测
        //Mat houghC = Hough_Circle(canny);
        //imshow("houghC",houghC);
    
        geometry_msgs::Twist cmd_red;

        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
    
        pub.publish(cmd_red);
    
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}