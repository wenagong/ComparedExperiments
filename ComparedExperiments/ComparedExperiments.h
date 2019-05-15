#pragma once
#include<opencv2/opencv.hpp>
using namespace cv;
Mat intrinsicMatrix1;
Mat intrinsicMatrix2;
Mat R;
Mat T; 
Mat distor1;
Mat distor2; 
int picWidth = 1296;
int picHeight = 966;

typedef struct _PixelPoint {
	double xl, yl, xr, yr, point_x, point_y, point_z;
	_PixelPoint* pNext;
}PixelPoint;

typedef struct _PixelPointPair {
	_PixelPoint* PixelPoints;
}PixelPointPair;

void JudgeOverlap();
void ProjectionPoint(Point3d thisPoint3d, Point2d& rPoint2dl, Point2d& rPoint2dr, Mat, Mat, Mat, Mat, Mat, Mat);
void ReadCalibParams(Mat& intrinsicMatrix1, Mat& intrinsicMatrix2, Mat& R, Mat& T, Mat& distor1, Mat& distor2, int& width, int& height);
