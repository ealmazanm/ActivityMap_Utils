#include "ActivityMap_Utils.h"

ActivityMap_Utils::ActivityMap_Utils(int ns):NUM_SENSORS(ns)
{	
	DEPTH_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  abs(MIN_X-MAX_X);

	XRes = ceil(double(YRes)*(double(X_VAR)/double(DEPTH_VAR)));

	depthStep = ceil(double(DEPTH_VAR)/double(YRes));
	xStep = ceil(double(X_VAR)/double(XRes));
}

ActivityMap_Utils::ActivityMap_Utils(void):NUM_SENSORS(2)
{
	DEPTH_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  abs(MIN_X-MAX_X);

	XRes = ceil(double(YRes)*(double(X_VAR)/double(DEPTH_VAR)));
}

ActivityMap_Utils::~ActivityMap_Utils(void)
{
}


Mat* ActivityMap_Utils::createActivityMap(KinectSensor* kinects, const XnDepthPixel** depthMaps, const XnRGB24Pixel** rgbMaps, bool trans)
{
	//Fits the noncalibrated map in an image
	int shifted = 0;
	if (!trans)
		shifted = 3400;

	Mat* activityMap = new Mat(Size(XRes, YRes), CV_8UC3);
	Utils::initMat3u(*activityMap, 255);
	Mat highestMap (YRes, XRes, CV_32F);
	Utils::initMatf(highestMap, -50000);
	XnPoint3D** allRealPoints = new XnPoint3D*[NUM_SENSORS];
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		//create list of xnPoint3D
		XnPoint3D* depthPoints = convert2Points(depthMaps[i]);
		XnPoint3D* realPoints = kinects[i].arrayBackProject(depthPoints);
		if (trans)
			kinects[i].transformArray(realPoints);

		allRealPoints[i] = realPoints;
		delete[] depthPoints;
	}


	//Go through all sensors
	for (int iter = 0; iter < NUM_SENSORS; iter++)
	{
		//go through all points
		for (int i = 0; i < XN_VGA_Y_RES; i++)
		{
			for (int j = 0; j < XN_VGA_X_RES; j++)
			{
				XnPoint3D p = allRealPoints[iter][i*XN_VGA_X_RES+j];
				if (p.Z > 0 && p.Z < MAX_Z && p.Y < 0) // threshold the ceiling (hopefully)
				{
					int xCoor = findCoordinate((p.X-shifted), MIN_X, MAX_X, xStep);
					int yC = findCoordinate(p.Z, MIN_Z, MAX_Z, depthStep);
					int yCoor = (YRes-1) - yC;

					float* ptr_h = highestMap.ptr<float>(yCoor);
				
					if (p.Y > ptr_h[xCoor])
					{
						ptr_h[xCoor] = p.Y;
						XnRGB24Pixel color = rgbMaps[iter][i*XN_VGA_X_RES+j];
						if (!trans)
						{
							circle(*activityMap, Point((XRes/NUM_SENSORS)*iter+xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
						}
						else
							circle(*activityMap, Point(xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
					}
				}
			}
		}
	}
		
	for (int iter = 0; iter < NUM_SENSORS; iter++)
		delete[] allRealPoints[iter];

	return activityMap;
}


//Private members
XnPoint3D* ActivityMap_Utils::convert2Points(const XnDepthPixel* depthMap)
{
	XnPoint3D* depthPoints = new XnPoint3D[XN_VGA_Y_RES*XN_VGA_X_RES];
	for (int i = 0; i < XN_VGA_Y_RES; i++)
	{
		for (int j = 0; j < XN_VGA_X_RES; j++)
		{
			depthPoints[i*XN_VGA_X_RES+j].X = j;
			depthPoints[i*XN_VGA_X_RES+j].Y = i;
			depthPoints[i*XN_VGA_X_RES+j].Z = depthMap[i*XN_VGA_X_RES+j];
		}
	}
	return depthPoints;
}