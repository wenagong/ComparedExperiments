#pragma once

int picWidth = 1396;
int picHeight = 966;

typedef struct _PixelPoint {
	double xl, yl, xr, yr, point_x, point_y, point_z;
	_PixelPoint* pNext;
}PixelPoint;

typedef struct _PixelPointPair {
	_PixelPoint* PixelPoints;
}PixelPointPair;

