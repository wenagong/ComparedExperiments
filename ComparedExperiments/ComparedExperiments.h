#pragma once
#include<opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat intrinsicMatrix1; //�������
Mat intrinsicMatrix2;
Mat R; //���������ת����
Mat T; //�������ƽ������
Mat distor1; //���������ϵ��(k1,k2,p1,p2)
Mat distor2; //���������ϵ��
int picWidth = 1296;
int picHeight = 966;
Mat Pl, Pr; //����У���������ͶӰ����3X4
Mat iRl(3, 3, CV_64F);
Mat iRr(3, 3, CV_64F);
Mat Plt(3, 3, CV_64F); //����У����������������3X3
Mat Prt(3, 3, CV_64F);
double Threshold2D = 2.0; //���ؾ�����ֵ
double Threshold3D = 1.0; //���Ƽ���ֵ

int same, differ, bl;
ofstream debug,debug1;

typedef struct _PixelPoint {
	double xl, yl, xr, yr, point_x, point_y, point_z, e;
	_PixelPoint* pNext;
}PixelPoint;

typedef struct _PixelPointPair {
	_PixelPoint* PixelPoints;
}PixelPointPair;

void JudgeOverlap();
void ProjectionPoint(Point3d thisPoint3d, Point3d thisPoint3d1, Point2d thisPoint2dl,Point2d& rPoint2dl, Point2d& rPoint2dr, double&eThis);
void ReadCalibParams(Mat& intrinsicMatrix1, Mat& intrinsicMatrix2, Mat& R, Mat& T, Mat& distor1, Mat& distor2, int& width, int& height);
void GetProjectionmatrix();
void myDistPoints(Point2d src, Point2d &dst, Mat& cameraMatrix, Mat& distortionCoeff);
void CalculateError(Point3d P3, Point P2, double& e);
