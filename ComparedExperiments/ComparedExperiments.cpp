// ComparedExperiments.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include"ComparedExperiments.h"
#include <iostream>

using namespace std;
using namespace cv;

void JudgeOverlap() {
	ReadCalibParams(intrinsicMatrix1, intrinsicMatrix2, R, T, distor1, distor2, picWidth, picHeight);
	//二维数组初始化
	PixelPointPair** LastPixelPairs = new PixelPointPair*[picHeight];
	for (int i = 0; i < picHeight; i++) {
		LastPixelPairs[i] = new PixelPointPair[picWidth];
		for (int j = 0; j < picWidth; j++) {
			LastPixelPairs[i][j].PixelPoints = NULL;
		}
	}

	ifstream L2DL, L2DR, L3DL,L3DR;
	L2DL.open("Datas\\Sequence01\\MatchResult2D_LeftPoints.TXT");
	L2DR.open("Datas\\Sequence01\\MatchResult2D_RightPoints.TXT");
	L3DL.open("Datas\\PointCloud3D01.TXT");
	L3DR.open("Datas\\PointCloud3D01.TXT");

	int lastNums=-1;
	char str[200];
	while (!L3DL.eof()) {
		L3DL.getline(str, sizeof(str));
		lastNums++;
	}
	L3DL.clear();
	L3DL.seekg(0, ios::beg);
	
	int thisNums = -1;
	while (!L3DR.eof()) {
		L3DR.getline(str, sizeof(str));
		thisNums++;
	}
	L3DR.clear();
	L3DR.seekg(0, ios::beg);

	int p;
	Vec2d pl, pr;
	Vec3d p3d;
	for (p = 0; p < lastNums; p++) {  //上一摄站获取数据存入二维数组
		L2DL >> pl[0];
		L2DL >> pl[1];

		L2DR >> pr[0];
		L2DR >> pr[1];

		L3DL >> p3d[0];
		L3DL >> p3d[1];
		L3DL >> p3d[2];

		int cols = (int)pl[0];
		int rows = (int)pl[1];
		PixelPoint* P = new PixelPoint;
		P->xl = pl[0];
		P->yl = pl[1];
		P->xr = pr[0];
		P->yr = pr[1];
		P->point_x = p3d[0];
		P->point_y = p3d[1];
		P->point_z = p3d[2];

		if (LastPixelPairs[rows][cols].PixelPoints == NULL) {
			LastPixelPairs[rows][cols].PixelPoints = P;
			P->pNext = NULL;
		}
		else {
			P->pNext = LastPixelPairs[rows][cols].PixelPoints;
			LastPixelPairs[rows][cols].PixelPoints = P;
		}
	}

	double tx, ty, tz;
	Point3d thisPoint3d;
	Point2d rPoint2dl, rPoint2dr;
	for (p = 0; p < thisNums; p++) {   //遍历此次摄站获取点云
		L3DR >> p3d[0];
		L3DR >> p3d[1];
		L3DR >> p3d[2];

		thisPoint3d.x = p3d[0];
		thisPoint3d.y = p3d[1];
		thisPoint3d.z = p3d[2];

		ProjectionPoint(thisPoint3d, rPoint2dl, rPoint2dr, intrinsicMatrix1, intrinsicMatrix2,R,T,distor1,distor2);
	}

}

void ReadCalibParams(Mat& intrinsicMatrix1,Mat& intrinsicMatrix2,Mat& R,Mat& T,Mat& distor1,Mat& distor2,int& width,int& height) {

	double fx1, fy1, cx1, cy1, dist11, dist12, dist13, dist14;
	double fx2, fy2, cx2, cy2, dist21, dist22, dist23, dist24;
	double r1, r2, r3, t1, t2, t3;
	ifstream calib;
	calib.open("Datas\\CalibParam.txt");
	calib >> fx1;
	calib >> fy1;
	calib >> cx1;
	calib >> cy1;
	calib >> dist11;
	calib >> dist12;
	calib >> dist13;
	calib >> dist14;
	calib >> fx2;
	calib >> fy2;
	calib >> cx2;
	calib >> cy2;
	calib >> dist21;
	calib >> dist22;
	calib >> dist23;
	calib >> dist24;
	calib >> r1;
	calib >> r2;
	calib >> r3;
	calib >> t1;
	calib >> t2;
	calib >> t3;
	calib >> width;
	calib >> height;
	intrinsicMatrix1 = (Mat_<double>(3, 3) << fx1, 0, cx1, 0, fy1, cy1, 0, 0, 1);
	intrinsicMatrix2 = (Mat_<double>(3, 3) << fx2, 0, cx2, 0, fy2, cy2, 0, 0, 1);
	R = (Mat_<double>(3, 1) << r1, r2, r3);
	T = (Mat_<double>(3, 1) << t1, t2, t3);
	distor1 = (Mat_<double>(4, 1) << dist11, dist12, dist13, dist14);
	distor2 = (Mat_<double>(4, 1) << dist21, dist22, dist23, dist24);
}

void ProjectionPoint(Point3d thisPoint3d,Point2d& rPoint2dl, Point2d& rPoint2dr,Mat intrinsicMatrix1, Mat intrinsicMatrix2, Mat R, Mat T, Mat distor1, Mat distor2) {
	ofstream out1;
	out1.open("Datas\\out2Dl.txt");
	vector<Point3d>thisPoint;
	vector<Point2d>out2Dl;
	vector<Point2d>out2Dr;
	thisPoint.push_back(thisPoint3d);
	Mat Rl=(Mat_<double>(3, 1)<<0,0,0);
	Mat Tl = (Mat_<double>(3, 1) << 0, 0, 0);
	projectPoints(thisPoint, Rl, Tl, intrinsicMatrix1, distor1, out2Dl);
	out1 << intrinsicMatrix1<<"，"<<R<<"，"<<T << endl;
	cout << out2Dl << endl;
}

int main()
{
	JudgeOverlap();

}

