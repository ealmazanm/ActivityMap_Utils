#pragma once
#include "KinectSensor.h"
#include "Plane.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <list>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"

#define MAX_DEPTH 10000

using namespace std;
using namespace cv;
using namespace xn;

const int REF_CAM = 1;

//REF_CAM = 2

//values for data set workshp
//const int MIN_X = -8500;
//const int MAX_X = 8500;
//const int MIN_Z = 500;
//const int MAX_Z = 9500; 

//values for dataset lab all
//static const int MIN_X = -11000;
//static const int MAX_X = 11000;
//static const int MIN_Z = 500;
//static const int MAX_Z = 9500;

//const int MIN_X = -3500;
//const int MAX_X = 3500;
//const int MIN_Z = 500;
//const int MAX_Z = 5000;

//values for data single Kinect
//const int MIN_X = -5500;
//const int MAX_X = -1500;
//const int MIN_Z = 500;
//const int MAX_Z = 5800;


const int YRes = 480;


class ActivityMap_Utils
{
public:

	static int MIN_X;
	static int MAX_X;
	static int MIN_Z;
	static int MAX_Z;

	ActivityMap_Utils(int);
	ActivityMap_Utils(void);
	~ActivityMap_Utils(void);

	void createActivityMap(KinectSensor* kinects, const XnDepthPixel** depthMaps, const XnRGB24Pixel** rgbMaps, bool trans, Mat& background, int nFrame);

	inline Size getResolution()
	{
		return Size(XRes, YRes);
	}

	static inline int findCoordinate(float value, float minValue, float maxValue, double step)
	{
		if (value < minValue)
			value = minValue;
		if (value > maxValue)
			value = maxValue;
		return (int)floor((value-minValue)/step);
	}

	static inline int findCoordinate_inv(float value, float minValue, float maxValue, double step)
	{
		return (int)floor((value*step) + minValue + (step/2));
	}

	static void getImageBinned(const Mat* frame, Mat& binned, Size binSz, bool flag);

	static XnPoint3D* convert2Points(const XnDepthPixel*);

	int depthStep;
	int xStep;	

private:
	

	int XRes;

	const int NUM_SENSORS;

	int DEPTH_VAR;
	int X_VAR;

};