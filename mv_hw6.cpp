#include "opencv2/opencv.hpp"
#include <iostream>
#include <windows.h>

using namespace cv;
using namespace std;

void mergeImage(Mat& dst, vector<Mat>& images)
{
	int imgCount = (int)images.size();

	if (imgCount <= 0)
	{
		printf("the number of images is too small\n");
		return;
	}

	printf("imgCount = %d\n", imgCount);

	/*将每个图片缩小为指定大小*/
	int rows = 300;
	int cols = 400;
	for (int i = 0; i < imgCount; i++)
	{
		resize(images[i], images[i], Size(cols, rows));
	}

	dst.create(rows * imgCount / 2, cols * 2, CV_8UC3);

	for (int i = 0; i < imgCount; i++)
	{
		images[i].copyTo(dst(Rect((i % 2) * cols, (i / 2) * rows, images[0].cols, images[0].rows)));
	}
}

void findcircles(Mat& srcImage, Mat& dstImage)
{
	Mat grayImage, gaussian;
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	GaussianBlur(grayImage, gaussian, Size(5, 5), 2, 2);

	vector<Vec3f> circles;
	HoughCircles(gaussian, circles, HOUGH_GRADIENT, 1.5, 10, 200, 100, 0, 0);

	for (size_t i = 0; i < circles.size(); i++) {
		Point center(cvRound(circles[i][0]), cvRound(circles[0][1]));
		int radius = cvRound(circles[i][2]);
		circle(dstImage, center, radius, Scalar(0, 0, 255), 3, 8, 0);
		cout << "center: " << center << "   diameter: " << 2 * radius << "   area: " << 3.14 * radius * radius << endl;
	}
}

int main(int argc, char** argv)
{
	DWORD start_time = GetTickCount();
	{
		vector<Mat> srcImage(5), dstImage(6);
		Mat merge;
		srcImage[0] = imread("1.png");
		srcImage[1] = imread("2.png");
		srcImage[2] = imread("3.png");
		srcImage[3] = imread("4.png");
		srcImage[4] = imread("5.png");
		dstImage[0] = srcImage[0].clone();

		for (int i = 0; i < 5; i++) {
			dstImage[i + 1] = srcImage[i].clone();
			findcircles(srcImage[i], dstImage[i + 1]);
		}

		mergeImage(merge, dstImage);
		imshow("dst", merge);
	}
	DWORD end_time = GetTickCount();
	cout << "The run time is:" << (end_time - start_time) * 1.00 / 1000 << "s!" << endl;//输出运行时间

	waitKey(0);
	return 0;
}
