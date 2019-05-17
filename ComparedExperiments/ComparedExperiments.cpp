// ComparedExperiments.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include"ComparedExperiments.h"

using namespace std;
using namespace cv;

void ReadCalibParams(Mat& intrinsicMatrix1, Mat& intrinsicMatrix2, Mat& R, Mat& T, Mat& distor1, Mat& distor2, int& width, int& height) {

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

void GetProjectionmatrix() {
	ifstream L2DL, L2DR;
	L2DL.open("Datas\\Sequence02\\MatchResult2D_LeftPoints.TXT");
	L2DR.open("Datas\\Sequence02\\MatchResult2D_RightPoints.TXT");
	int lastNums = -1;
	char str[200];
	while (!L2DL.eof()) {
		L2DL.getline(str, sizeof(str));
		lastNums++;
	}
	L2DL.clear();
	L2DL.seekg(0, ios::beg);

	Mat _src1(lastNums, 1, CV_32FC2);
	Mat _src2(lastNums, 1, CV_32FC2);

	Vec2f dd;
	for (int i = 0; i < lastNums; i++) {
		L2DL >> dd[0];
		L2DL >> dd[1];
		_src1.at<Vec2f>(i, 0) = dd;
	}
	for (int i = 0; i < lastNums; i++) {
		L2DR >> dd[0];
		L2DR >> dd[1];
		_src2.at<Vec2f>(i, 0) = dd;
	}

	Mat _dst1;
	Mat _dst2;

	//畸变矫正(将像素坐标转换为了矫正过的图像坐标)
	undistortPoints(_src1, _dst1, intrinsicMatrix1, distor1); //校正后的坐标需要乘以焦距+中心坐标变为矫正后的像素坐标
	undistortPoints(_src2, _dst2, intrinsicMatrix2, distor2);

	Mat R2_matrix(Size(3, 3), CV_64F); //右相机旋转向量
	Rodrigues(R, R2_matrix);   //旋转向量转化为旋转矩阵

	//立体校正，校正后的立体相机光轴平行，且行逐行对齐
	Mat Rl, Rr, Q;
	stereoRectify(intrinsicMatrix1, distor1, intrinsicMatrix2, distor2, Size(1296, 966), R2_matrix, T, Rl, Rr, Pl, Pr, Q, 0, -1, Size(1296, 966));

}

void JudgeOverlap() {
	ReadCalibParams(intrinsicMatrix1, intrinsicMatrix2, R, T, distor1, distor2, picWidth, picHeight);
	GetProjectionmatrix();
	//二维数组初始化
	PixelPointPair** LastPixelPairs = new PixelPointPair*[picHeight];
	for (int i = 0; i < picHeight; i++) {
		LastPixelPairs[i] = new PixelPointPair[picWidth];
		for (int j = 0; j < picWidth; j++) {
			LastPixelPairs[i][j].PixelPoints = NULL;
		}
	}

	ifstream L2DL, L2DR, L3DL,T3D,T2DL,T3D1;
	L2DL.open("Datas\\Sequence01\\MatchResult2D_LeftPoints.TXT");
	L2DR.open("Datas\\Sequence01\\MatchResult2D_RightPoints.TXT");
	L3DL.open("Datas\\Sequence01\\Result3D_Points.TXT");
	T2DL.open("Datas\\Sequence02\\MatchResult2D_LeftPoints.TXT");
	T3D.open("Datas\\Sequence02\\Result3D_Points.TXT");
	T3D1.open("Datas\\PointCloud3D02.TXT");
	debug.open("debug.txt");
	int lastNums=-1;
	char str[200];
	while (!L3DL.eof()) {
		L3DL.getline(str, sizeof(str));
		lastNums++;
	}
	L3DL.clear();
	L3DL.seekg(0, ios::beg);
	
	int thisNums = -1;
	while (!T3D.eof()) {
		T3D.getline(str, sizeof(str));
		thisNums++;
	}
	T3D.clear();
	T3D.seekg(0, ios::beg);

	int p;
	Vec2d pl, pr,tpl;
	Vec3d p3d,p3d1;
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
	Point3d thisPoint3d, thisPoint3d1;  //未统一坐标/统一坐标
	Point2d rePoint2dl, rePoint2dr,tPoint2dl;
	for (p = 0; p < thisNums; p++) {   //遍历此次摄站获取点云
		T3D >> p3d[0];
		T3D >> p3d[1];
		T3D >> p3d[2];

		T3D1 >> p3d1[0];
		T3D1 >> p3d1[1];
		T3D1 >> p3d1[2];

		T2DL >> tpl[0];
		T2DL >> tpl[1];

		thisPoint3d.x = p3d[0];
		thisPoint3d.y = p3d[1];
		thisPoint3d.z = p3d[2];

		thisPoint3d1.x = p3d1[0];
		thisPoint3d1.y = p3d1[1];
		thisPoint3d1.z = p3d1[2];


		tPoint2dl.x = tpl[0];
		tPoint2dl.y = tpl[1];

		double eThis; //重投影残差
		ProjectionPoint(thisPoint3d,thisPoint3d1,tPoint2dl,rePoint2dl, rePoint2dr, eThis);
		int rows = (int)rePoint2dl.y;
		int cols = (int)rePoint2dl.x;
		
		double minThre = INT_MAX; 
		int Flag = 0;
		double x, y, z;
		for (int m = rows - 1; m <= rows + 1; m++) {
			for (int n = cols - 1; n <= cols + 1; n++) {
				while (LastPixelPairs[m][n].PixelPoints!=NULL) {
					PixelPoint* P = LastPixelPairs[m][n].PixelPoints;
					double xl = P->xl;
					double yl = P->yl;
					double xr = P->xr;
					double yr = P->yr;
					double Dl = sqrt(pow(rePoint2dl.x - xl, 2) + pow(rePoint2dl.y - yl, 2));
					double Dr = sqrt(pow(rePoint2dr.x - xr, 2) + pow(rePoint2dr.y - yr, 2));
					double D = Dl + Dr;
					if (Dl < Threshold2D&&Dr < Threshold2D&&D < minThre) { //约束1：重投影左右二维像点与上一摄站采集左右像点欧式距离同时小于给定阈值(最小)
						 x = P->point_x;
						 y = P->point_y;
						 z = P->point_z;
						Flag = 1;  //找到符合条件的点
					}
					LastPixelPairs[m][n].PixelPoints = LastPixelPairs[m][n].PixelPoints->pNext;
				}
			}
		}
		if (Flag == 1) {
			double D3d = sqrt(pow(p3d[0]-x, 2) + pow(p3d[1]-y, 2) + pow(p3d[2]-z, 2));
			if (D3d < Threshold3D) {  //约束2：三维点距离小于点云均值，根据重投影残差取舍

			}
		}
	}

}

void myDistPoints(Point2d src, Point2d &dst, Mat& cameraMatrix, Mat& distortionCoeff) {  //反畸变校正
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0,0);
	double k2 = distortionCoeff.at<double>(1, 0);
	double p1 = distortionCoeff.at<double>(2, 0);
	double p2 = distortionCoeff.at<double>(3, 0);
	double k3 = 0;
	double k4 = 0;//
	double k5 = 0;//
	double k6 = 0;//
	Point2d& p = src;

	//获取的点通常是图像的像素点，所以需要先通过小孔相机模型转换到归一化坐标系下；
	double xCorrected = (p.x - ux) / fx;
	double yCorrected = (p.y - uy) / fy;

	double xDistortion, yDistortion;
	//我们已知的是经过畸变矫正或理想点的坐标；
	double r2 = xCorrected * xCorrected + yCorrected * yCorrected;

	double deltaRa = 1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
	double deltaRb = 1 / (1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2);
	double deltaTx = 2. * p1 * xCorrected * yCorrected + p2 * (r2 + 2. * xCorrected * xCorrected);
	double deltaTy = p1 * (r2 + 2. * yCorrected * yCorrected) + 2. * p2 * xCorrected * yCorrected;

	//下面为畸变模型；
	xDistortion = xCorrected * deltaRa * deltaRb + deltaTx;
	yDistortion = yCorrected * deltaRa * deltaRb + deltaTy;

	//最后再次通过相机模型将归一化的坐标转换到像素坐标系下；
	xDistortion = xDistortion * fx + ux;
	yDistortion = yDistortion * fy + uy;

	dst.x = xDistortion;
	dst.y = yDistortion;
	//dst.push_back(Point2d(xDistortion, yDistortion));
}


void ProjectionPoint(Point3d thisPoint3d, Point3d thisPoint3d1,Point2d thisPoint2dl,Point2d& rePoint2dl, Point2d& rePoint2dr,double& eThis) {

	Mat tPoint3D = (Mat_<double>(4, 1) << -thisPoint3d.x, thisPoint3d.y, thisPoint3d.z, 1.0); //未统一摄站的点
	Mat rp(3, 1, CV_64F);
	rp = Pl * tPoint3D;
	Point2d rpsrc,rpdst;
	rpsrc.x= rp.at<double>(0, 0) / rp.at<double>(2, 0);
	rpsrc.y= rp.at<double>(1, 0) / rp.at<double>(2, 0);
	myDistPoints(rpsrc, rpdst, intrinsicMatrix1, distor1);
	
	vector<Point2d>p1, p2;
	p1.push_back(thisPoint2dl);
	p2.push_back(rpdst); 
	eThis = norm(p1, p2, CV_L2); //重投影残差

	Mat tPoint3D1 = (Mat_<double>(4, 1) << -thisPoint3d1.x, -thisPoint3d1.y, -thisPoint3d1.z, 1.0);  //统一摄站的点
	Mat rpl(3, 1, CV_64F);
	Mat rpr(3, 1, CV_64F);
	rpl = Pl * tPoint3D1;
	rePoint2dl.x = rpl.at<double>(0, 0) / rpl.at<double>(2, 0);
	rePoint2dl.y = rpl.at<double>(1, 0) / rpl.at<double>(2, 0);
	
	rpr = Pr * tPoint3D1;
	rePoint2dr.x = rpr.at<double>(0, 0) / rpr.at<double>(2, 0);
	rePoint2dr.y = rpr.at<double>(1, 0) / rpr.at<double>(2, 0);
}


int main()
{
	JudgeOverlap();
	
}

