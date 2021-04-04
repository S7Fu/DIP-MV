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

#define LINEAR_X 0

#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;

int hmin = 0, hmax = 180, smin = 0, smax = 255, vmin = 0, vmax = 255;
Mat hsvimg;

struct BGR{
    uchar b, g, r;
};

struct HSV{
    int h;
    double s, v;
};

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

////////////转换为色度空间//////////////////
bool IsEqual(double val1, double val2){
    return fabs(val1 - val2) < 0.001;
}

void BGR2HSV(BGR &bgr, HSV &hsv){
    double b, g, r;
    double h, s, v;
    double min, max;

    b = bgr.b/255.0;
    g = bgr.g/255.0;
    r = bgr.r/255.0;
    //得到h
    if(r > g){
        max = MAX(r, b);
        min = MIN(g, b);
    }
    else{
        max = MAX(g, b);
        min = MIN(r, b);
    }
    v = max;
    //得到s
    if(IsEqual(max, 0))
        s = 0.0;
    else
        s = 1 - min/max;
    //得到h
    if(max == min)
        h = 0.0;
    else{
        if(IsEqual(r, max) && g >= b)
            h = 60*(g-b)/(max - min) + 0;
        else if(IsEqual(r, max) && g < b)
            h = 60*(g-b)/(max - min) + 360;
        else if(IsEqual(max, g))
            h = 60*(b-r)/(max - min) + 120;
        else if(IsEqual(max, b))
            h = 60*(r-g)/(max - min) + 240;
    }
    hsv.h = (int)(h + 0.5);
    hsv.h = (hsv.h > 359) ? (hsv.h - 360) : hsv.h;
    hsv.h = (hsv.h < 0) ? (hsv.h + 360) : hsv.h;
    hsv.s = s;
    hsv.v = v;
}

Mat tohsv(Mat input){
    Mat output = input.clone();
    BGR bgr;
    HSV hsv;
    for(int i = 0; i < input.rows; i++)
    {
        uchar* pinput = input.ptr<uchar>(i);
        uchar* poutput = output.ptr<uchar>(i);
        for(int j = 0; j < input.cols*3; j+=3)
        {
            bgr.b = pinput[j];
            bgr.g = pinput[j+1];
            bgr.r = pinput[j+2];
            BGR2HSV(bgr, hsv);
            poutput[j] = (uchar)180*hsv.h/360;
            poutput[j+1] = (uchar)255*hsv.s;
            poutput[j+2] = (uchar)255*hsv.v;
        }
    }
    return output;
}

///////////颜色分割//////////////////
void ColorSegment(int, void*){
    Mat output = Mat::zeros(hsvimg.size(), hsvimg.type());
    inRange(hsvimg, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), output);
    imshow("ColorSegment", output);
}

/////////////形态学//////////////////
// 膨胀函数
Mat Dilate(Mat input, int n){
	double **templateM = new double*[n];
	for(int i = 0; i < n; i++)
		templateM[i] = new double[n];
	int center = n/2;   //以模板的中心为原点
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
			templateM[i][j] = 255;
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
				output.at<uchar>(i, j) = 255;
			else
				output.at<uchar>(i, j) = 0;
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
			templateM[i][j] = 255;
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
			if(sum == n*n)
				output.at<uchar>(i, j) = 255;
			else
				output.at<uchar>(i, j) = 0;
		}
	}
	for(int i = 0; i < n; i++)
		delete[] templateM[i];
	delete[] templateM;
	return output;
}

void ShowColor(Mat input){
    Mat R, R_, G, B, Y;
    inRange(input, Scalar(0, 43, 46), Scalar(10, 255, 255), R);
    inRange(input, Scalar(156, 43, 46), Scalar(180, 255, 255), R_);
    for(int i = 0; i < input.rows; i++)
        for(int j = 0; j < input.cols; j++)
            if(R_.at<uchar>(i,j) == 255)
                R.at<uchar>(i,j) = R_.at<uchar>(i,j);
    inRange(input, Scalar(35, 43, 46), Scalar(77, 255, 255), G);
    inRange(input, Scalar(100, 43, 46), Scalar(124, 255, 255), B);
    inRange(input, Scalar(26, 43, 46), Scalar(34, 255, 255), Y);
    imshow("R", R);
    imshow("G", G);
    imshow("B", B);
    imshow("Y", Y);
    int sum_R=0, sum_G=0, sum_B=0, sum_Y=0;
    for(int i = 0; i < R.rows; i++)
        for(int j = 0; j < R.cols; j++)
            if(R.at<uchar>(i,j) == 255)
                sum_R += 1;
    
    for(int i = 0; i < G.rows; i++)
        for(int j = 0; j < G.cols; j++)
            if(G.at<uchar>(i,j) == 255)
                sum_G += 1;
    //cout << sum_G << "         " << G.rows * G.cols << endl;
    for(int i = 0; i < B.rows; i++)
        for(int j = 0; j < B.cols; j++)
            if(B.at<uchar>(i,j) == 255)
                sum_B += 1;
    for(int i = 0; i < Y.rows; i++)
        for(int j = 0; j < Y.cols; j++)
            if(Y.at<uchar>(i,j) == 255)
                sum_Y += 1;
    //利用Mat类新建一个600×800的黑色画布，在该画布上绘制直方图。
	Mat hist = Mat::zeros(600, 800, CV_8UC3);
    rectangle(hist, Point(0,800), Point(0 + 100, 800 - 800 * sum_R/(input.rows * input.cols)), Scalar(0, 0, 255), 2, 8);
    rectangle(hist, Point(100,800), Point(100 + 100, 800 - 800 * sum_G/(input.rows * input.cols)), Scalar(0, 255, 0), 2, 8);
    rectangle(hist, Point(200,800), Point(200 + 100, 800 - 800 * sum_B/(input.rows * input.cols)), Scalar(255, 0, 2), 2, 8);
    rectangle(hist, Point(300,800), Point(300 + 100, 800 - 800 * sum_Y/(input.rows * input.cols)), Scalar(255, 255, 0), 2, 8);
    imshow("hist", hist);
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
    //Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/color.png");
    //Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/RGB.jpeg");
    Mat frame = imread("/home/s7fu/catkin_ws/src/image_pkg/src/redball.jpg");
    Mat blur = frame.clone();
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
        
        // 高斯滤波
        imshow("frame", frame);
        Mat src_channels[3];
        Mat blur_channels[3];
        split(frame, src_channels);
        for(int i = 0; i < 3; i++){
            blur_channels[i] = Gaussian(src_channels[i], 3, 0.8);
        }
        merge(blur_channels, 3, blur);
        imshow("blur", blur);

        //GaussianBlur(frame, blur, Size(5, 5), 3, 3);

        //RGB转HSV
        hsvimg = tohsv(blur);
        imshow("hsv", hsvimg);
        /*
        hsv = blur.clone();
        cvtColor(hsv, hsv, CV_BGR2HSV);
        imshow("hsv", hsv);
        */

        //颜色分割
        
        Mat R, R_, G, B;
        inRange(hsvimg, Scalar(0, 43, 46), Scalar(10, 255, 255), R);
        inRange(hsvimg, Scalar(156, 43, 46), Scalar(180, 255, 255), R_);
        for(int i = 0; i < hsvimg.rows; i++)
            for(int j = 0; j < hsvimg.cols; j++)
                if(R_.at<uchar>(i,j) == 255)
                    R.at<uchar>(i,j) = R_.at<uchar>(i,j);
        inRange(hsvimg, Scalar(35, 43, 46), Scalar(77, 255, 255), G);
        inRange(hsvimg, Scalar(100, 43, 46), Scalar(124, 255, 255), B);
        imshow("R", R);
        //imshow("G", G);
        //imshow("B", B);
        
        /*
        namedWindow("ColorSegment");
        createTrackbar("Hmax", "ColorSegment", &hmax, 180, ColorSegment);
        createTrackbar("Hmin", "ColorSegment", &hmin, 180, ColorSegment);
        createTrackbar("Smax", "ColorSegment", &smax, 255, ColorSegment);
        createTrackbar("Smin", "ColorSegment", &smin, 255, ColorSegment);
        createTrackbar("Vmax", "ColorSegment", &vmax, 255, ColorSegment);
        createTrackbar("Vmin", "ColorSegment", &vmin, 255, ColorSegment);
        */
        //目标颜色检测
        // 腐蚀
		Mat erode = Erode(R, 7);
		imshow("erode", erode);
        // 膨胀
        Mat dilate = Dilate(erode, 7);
		imshow("dilate", dilate);
        //进行轮廓检测
        Mat detect = frame.clone();
        vector<vector<Point> > contours;
        findContours(dilate, contours, noArray(), RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        vector<vector<Point> > contours_poly(contours.size());
        
        int max = 0;
        for (int i = 1; i<contours.size(); i++)
        {
            //approxPolyDP(Mat(contours[i]), contours_poly[i], 25, true);
            //drawContours(detect, contours_poly, i, Scalar(255, 0, 0), 2, 8);  //绘制
            if(abs(contourArea(contours[i])) > abs(contourArea(contours[max])))
                max = i;
        }
        Point2f center;
        float radius;
        minEnclosingCircle(Mat(contours[max]), center, radius);
        circle(detect, static_cast<Point>(center), (int)radius, Scalar(255, 0, 0), 2);
        imshow("detect", detect);

        //统计颜色
        Rect rect = boundingRect(Mat(contours[max]));
        Mat image_roi = frame(rect);
        imshow("roi", image_roi);

        Mat hist = tohsv(image_roi);
        ShowColor(hist);

        //geometry_msgs::Twist cmd_red;

        // 车的速度值设置
        /*
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
    
        pub.publish(cmd_red);
        */
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}