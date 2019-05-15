// ComparedExperiments.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include"ComparedExperiments.h"
#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void JudgeOverlap() {

	//二维数组初始化
	PixelPointPair** LastPixelPairs = new PixelPointPair*[picHeight];
	for (int i = 0; i < picHeight; i++) {
		LastPixelPairs[i] = new PixelPointPair[picWidth];
		for (int j = 0; j < picWidth; j++) {
			LastPixelPairs[i][j].PixelPoints = NULL;
		}
	}

	ifstream L2DL, L2DR, L3D;
	L2DL.open("Datas\\Sequence01\\MatchResult2D_LeftPoints.TXT");
	L2DR.open("Datas\\Sequence01\\MatchResult2D_RightPoints.TXT");
	L3D.open("Datas\\Sequence01\\Result3D_Points.TXT");

	int lastNums=-1;
	char str[200];
	while (!L3D.eof()) {
		L3D.getline(str, sizeof(str));
		lastNums++;
	}
	L3D.clear();
	L3D.seekg(0, ios::beg);

	Vec2f pl, pr;
	Vec3f p3d;
	for (int p = 0; p < lastNums; p++) {
		L2DL >> pl[0];
		L2DL >> pl[1];

		L2DR >> pr[0];
		L2DR >> pr[1];

		L3D >> p3d[0];
		L3D >> p3d[1];
		L3D >> p3d[2];

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

}


int main()
{
	

}

