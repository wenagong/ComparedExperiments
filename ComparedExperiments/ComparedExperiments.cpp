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

	Mat R2_matrix(Size(3, 3), CV_64F); //右相机旋转向量
	Rodrigues(R, R2_matrix);   //旋转向量转化为旋转矩阵

	//立体校正，校正后的立体相机光轴平行，且行逐行对齐
	Mat Rl, Rr, Q;
	stereoRectify(intrinsicMatrix1, distor1, intrinsicMatrix2, distor2, Size(picWidth, picHeight), R2_matrix, T, Rl, Rr, Pl, Pr, Q, 0, -1, Size(picWidth, picHeight));
	
    //分别投影原立体相机图像坐标到校正后立体相机图像坐标
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			iRl.at<double>(i, j) = Pl.at<double>(i, j);//取Pl的-2列所构成的*3矩阵与Rl相乘获得从原左相机平面图像到矫正后左相机平面图像的转换矩阵
			iRr.at<double>(i, j) = Pr.at<double>(i, j);
			Plt.at<double>(i, j) = Pl.at<double>(i, j);
			Prt.at<double>(i, j) = Pr.at<double>(i, j);
		}
	}
	iRl = iRl * Rl;
	iRr = iRr * Rr;

}

void JudgeOverlap() {

	ReadCalibParams(intrinsicMatrix1, intrinsicMatrix2, R, T, distor1, distor2, picWidth, picHeight);
	GetProjectionmatrix();

	//二维数组初始化，带有一个头指针
	PixelPointPair** LastPixelPairs = new PixelPointPair*[picHeight];
	for (int i = 0; i < picHeight; i++) {
		LastPixelPairs[i] = new PixelPointPair[picWidth];
		for (int j = 0; j < picWidth; j++) {
			LastPixelPairs[i][j].PixelPoints = new PixelPoint;
			LastPixelPairs[i][j].PixelPoints->pNext = NULL;
		}
	}

	ifstream L2DL, L2DR, L3DL, T3D, T2DL, T3D1;
	L2DL.open("Datas\\Sequence01\\MatchResult2D_LeftPoints.TXT");
	L2DR.open("Datas\\Sequence01\\MatchResult2D_RightPoints.TXT");
	L3DL.open("Datas\\Sequence01\\Result3D_Points.TXT");
	T2DL.open("Datas\\Sequence02\\MatchResult2D_LeftPoints.TXT");
	T3D.open("Datas\\Sequence02\\Result3D_Points.TXT");
	T3D1.open("Datas\\PointCloud3D02.TXT");  //二摄站下统一全局坐标
	debug.open("debug.txt");
	debug1.open("debug1.txt");

	ofstream Last3D, This3D;
	Last3D.open("Datas\\Last3D.txt");
	This3D.open("Datas\\This3D.txt");

	int lastNums = -1;
	char str[200];
	while (!L3DL.eof()) {
		L3DL.getline(str, sizeof(str));
		lastNums++; //上一摄站点云数目
	}
	L3DL.clear();
	L3DL.seekg(0, ios::beg);

	int thisNums = -1;
	while (!T3D.eof()) {
		T3D.getline(str, sizeof(str));
		thisNums++;  //这一摄站点云数目
	}
	T3D.clear();
	T3D.seekg(0, ios::beg);

	int p;
	Vec2d pl, pr, tpl;
	Vec3d p3d, p3d1;
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
		P->point_x = p3d[0]; //原始点云数据是wx,-wy,-wz要变为wx,wy,wz
		P->point_y = -p3d[1];
		P->point_z = -p3d[2];

		Point3d lastP3d(p3d[0], -p3d[1], -p3d[2]);  
		Point2d lastP2dl(pl[0], pl[1]);
		double e1;
		CalculateError(lastP3d, lastP2dl, e1);  //计算重投影残差
		P->e = e1;

		if (LastPixelPairs[rows][cols].PixelPoints->pNext == NULL) {
			LastPixelPairs[rows][cols].PixelPoints->pNext = P;
			P->pNext = NULL;
			//differ++;
		}
		else {
			P->pNext = LastPixelPairs[rows][cols].PixelPoints->pNext; //头插法
			LastPixelPairs[rows][cols].PixelPoints->pNext = P;
			//same++;
		}
	}
	//debug << "same nums:" << same << "; differ nums:" << differ << endl;

	Point3d thisPoint3d, thisPoint3d1;  //未统一坐标/统一坐标(全局坐标系下点)
	Point2d rePoint2dl, rePoint2dr, tPoint2dl; //重投影左\右像点,此个左像点(求残差)
	int lNums = 0;
	int tNums = 0;
	int overlapNums=0;
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
		thisPoint3d.y = -p3d[1];
		thisPoint3d.z = -p3d[2];

		thisPoint3d1.x = p3d1[0];
		thisPoint3d1.y = -p3d1[1];
		thisPoint3d1.z = -p3d1[2];

		tPoint2dl.x = tpl[0];
		tPoint2dl.y = tpl[1];

		double eThis; //重投影残差
		ProjectionPoint(thisPoint3d, thisPoint3d1, tPoint2dl, rePoint2dl, rePoint2dr, eThis);   //求反投影左右像面二维坐标以及残差值大小
		int rows = (int)rePoint2dl.y; //根据重投影左像点的x,y索引到对应二维数组中
		int cols = (int)rePoint2dl.x;

		double minThre = INT_MAX;
		int Flag = 0; //满足约束一标记
		double x = 0;
		double y = 0;
		double z = 0;
		double eLast;
		PixelPoint* S = NULL; //S指向P的头结点
		PixelPoint* R = NULL; //R指向满足条件的结点
		if (rows == 0 || rows >= picHeight || cols == 0 || cols >= picWidth) {
			This3D << p3d1[0] << "  " << p3d1[1] << "  " << p3d1[2] << endl;
			continue;
		}
		for (int m = rows - 1; m <= rows + 1; m++) {
			for (int n = cols - 1; n <= cols + 1; n++) {
				PixelPoint* P = LastPixelPairs[m][n].PixelPoints->pNext;
				S = LastPixelPairs[m][n].PixelPoints; 
				while (P != NULL) {
					double xl = P->xl;
					double yl = P->yl;
					double xr = P->xr;
					double yr = P->yr;
					double Dl = sqrt(pow(rePoint2dl.x - xl, 2) + pow(rePoint2dl.y - yl, 2));
					double Dr = sqrt(pow(rePoint2dr.x - xr, 2) + pow(rePoint2dr.y - yr, 2));
					double D = Dl + Dr;
					if (Dl <= Threshold2D && Dr <= Threshold2D && D <= minThre) { //约束1：重投影左右二维像点与上一摄站采集左右像点欧式距离同时小于给定阈值(最小)
						x = P->point_x;
						y = P->point_y;
						z = P->point_z;
						eLast = P->e;
						R = P;
						//S->pNext = P; //指针S后面指向满足条件的结点
						Flag = 1;
					}
					P = P->pNext;
				}
			}
		}
		if (Flag == 1) { //找到满足约束一的最近点
			double D3d = sqrt(pow(thisPoint3d1.x - x, 2) + pow(thisPoint3d1.y - y, 2) + pow(thisPoint3d1.z - z, 2)); //求三维点间距大小
			if (D3d <= Threshold3D) {  //约束2：三维点距离小于点云均值，根据重投影残差取舍
				overlapNums++;
				debug1 << "eThis=" << eThis << "; eLast=" << eLast << endl;
				if (eThis <= eLast) { //如果当前点误差小于上一摄站点云，输出当前点，并且从链表中删除上一摄站链表中R指向的点
					tNums++;
					This3D << p3d1[0] << "  " << p3d1[1] << "  " << p3d1[2] << endl;
					//删除R指向的结点
					if (S->pNext == R) {
						S->pNext = R->pNext;
						free(R);
					}
					else {
						S = S->pNext;
					}
					//if (S->pNext->pNext == NULL) {
					//	PixelPoint*Q = S->pNext;
					//	S->pNext = NULL;
					//	free(Q);
					//}
					//else {
					//	PixelPoint*Q = S->pNext;  //删除链表中S指向的点(精度较低的重复点)
					//	S->e = Q->e;
					//	S->point_x = Q->point_x;
					//	S->point_y = Q->point_y;
					//	S->point_z = Q->point_z;
					//	S->xl = Q->xl;
					//	S->yl = Q->yl;
					//	S->xr = Q->xr;
					//	S->yr = Q->yr;
					//	S->pNext = Q->pNext;
					//	free(Q);
					//}
				}
				else if(eThis > eLast){
					lNums++;
					Last3D << x << "  " << y << "  " << z << "  " << endl;
					//删除链表中R指向的点(已经输出所以删除)
					if (S->pNext == R) {
						S->pNext = R->pNext;
						free(R);
					}
					else {
						S = S->pNext;
					}
				}
			}
			else if(D3d > Threshold3D) {   //不满足约束二，输出本次点云
				This3D << p3d1[0] << "  " << p3d1[1] << "  " << p3d1[2] << endl;
			}
		}
		else if (Flag == 0) { //没有找到满足约束条件的点，即没有重复发生
			This3D << p3d1[0] << "  " << p3d1[1] << "  " << p3d1[2] << endl;
		}
	}

	//遍历二维数组，输出剩余的上一摄站所有点云
	for (int i = 0; i < picHeight; i++) {
		for (int j = 0; j < picWidth; j++) {
			PixelPoint* temP = LastPixelPairs[i][j].PixelPoints->pNext;
			while (temP != NULL) { 
				Last3D << temP->point_x << "  " << temP->point_y << "  "
					<< temP->point_z << "  " << endl;
				temP = temP->pNext;
				bl++;
			}
		}
	}
	debug << "  bl:" << bl <<"; overlapNums:"<< overlapNums<<"; lNums:"<< lNums<<"; tNums:"<< tNums <<endl;
	debug, debug1,L2DL, L2DR, L3DL, T3D, T2DL, T3D1.close();
}

void myDistPoints(Point2d src, Point2d &dst, Mat& cameraMatrix, Mat& distortionCoeff) {  //反畸变校正
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0, 0);
	double k2 = distortionCoeff.at<double>(1, 0);
	double p1 = distortionCoeff.at<double>(2, 0);
	double p2 = distortionCoeff.at<double>(3, 0);
	double k3 = 0;
	double k4 = 0;
	double k5 = 0;
	double k6 = 0;
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

void CalculateError(Point3d P3, Point P2, double& e) {  //计算重投影残差
	
	//ifstream L2DL, L2DR;
	//L2DL.open("Datas\\Sequence02\\MatchResult2D_LeftPoints.TXT");
	//L2DR.open("Datas\\Sequence02\\MatchResult2D_RightPoints.TXT");
	//int lastNums = -1;
	//char str[200];
	//while (!L2DL.eof()) {
	//	L2DL.getline(str, sizeof(str));
	//	lastNums++;
	//}
	//L2DL.clear();
	//L2DL.seekg(0, ios::beg);

	Vec2f tem1;
	Mat _src(1, 1, CV_32FC2);
	tem1[0] = P2.x;
	tem1[1] = P2.y;
	_src = tem1;
	Mat _dst;

	//畸变矫正(将像素坐标转换为了矫正过的图像坐标)
	undistortPoints(_src, _dst, intrinsicMatrix1, distor1); //校正后的坐标需要乘以焦距+中心坐标变为矫正后的像素坐标
	tem1 = _dst.at<Vec2f>(0, 0);
	double x_ = tem1[0];
	double y_ = tem1[1];
	Mat xx = (Mat_<double>(3, 1) << x_, y_, 1);
	Mat xx1(3, 1, CV_64F);
	xx1 = iRl * xx;
	double x1 = xx1.at<double>(0, 0) / xx1.at<double>(2, 0);
	double y1 = xx1.at<double>(1, 0) / xx1.at<double>(2, 0);

	vector<Point3d>thisPoint3d;
	thisPoint3d.push_back(P3);
	Mat TSelf = (Mat_<double>(3, 1) << 0, 0, 0);   //平移向量
	Mat RSelf = (Mat_<double>(3, 1) << 0, 0, 0);
	vector<Point2d>res_2dl; //重投影坐像二维点
	projectPoints(thisPoint3d, RSelf, TSelf, Plt, distor1, res_2dl);  //重投影函数，Plt为立体校正得出的相机矩阵！

	//vector<Point2d>p1, p2;
	//p1.push_back(rpdst);
	//p2.push_back(P2);
	//e = norm(p1, p2, CV_L2);
	e = sqrt(pow(x1 - res_2dl[0].x, 2) + pow(y1 - res_2dl[0].y, 2));
	debug1 << "e=" << e<<endl;
}

void ProjectionPoint(Point3d thisPoint3d, Point3d thisPoint3d1, Point2d thisPoint2dl, Point2d& rePoint2dl, Point2d& rePoint2dr, double& eThis) {

	CalculateError(thisPoint3d, thisPoint2dl, eThis);//计算残差

	debug1 << "thisPoint3d=" << thisPoint3d << " thisPoint2dl=" << thisPoint2dl  << " eThis=" << eThis << endl;

	Mat tPoint3D1 = (Mat_<double>(4, 1) << thisPoint3d1.x, thisPoint3d1.y, thisPoint3d1.z, 1.0);  //统一摄站的点
	Mat rpl(3, 1, CV_64F);
	Mat rpr(3, 1, CV_64F);
	Point2d tem1, tem2;
	rpl = Pl * tPoint3D1; //立体校正得到的左像投影矩阵乘求得三维点得到左像校正后的像素点
	//左像反立体校正，将平面点转换到原始平面
	Mat preiRl(3, 3, CV_64F);
	Mat preiRr(3, 3, CV_64F);
	preiRl = iRl.inv(); //立体校正求得旋转矩阵的逆矩阵
	preiRr = iRr.inv();
	rpl = preiRl * rpl; 
	rpl = intrinsicMatrix1 * rpl;
	tem1.x = rpl.at<double>(0, 0) / rpl.at<double>(2, 0);
	tem1.y = rpl.at<double>(1, 0) / rpl.at<double>(2, 0);
	//反畸变校正
	myDistPoints(tem1, rePoint2dl, intrinsicMatrix1, distor1); //得到未经过畸变校正的点
	
	//右边反校正
	rpr = Pr * tPoint3D1;
	rpr = preiRr * rpr;
	rpr = intrinsicMatrix2 * rpr;
	tem2.x = rpr.at<double>(0, 0) / rpr.at<double>(2, 0);
	tem2.y = rpr.at<double>(1, 0) / rpr.at<double>(2, 0);
	myDistPoints(tem2, rePoint2dr, intrinsicMatrix2, distor2);

	debug << "thisPoint3d=" << thisPoint3d << " rePoint2dl=" << rePoint2dl << "  rePoint2dr=" << rePoint2dr << endl;
}

int main()
{
	JudgeOverlap();
}

