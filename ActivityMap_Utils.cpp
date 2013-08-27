#include "ActivityMap_Utils.h"


//values for dataset lab all
//static const int MIN_X = -11000;
//static const int MAX_X = 11000;
//static const int MIN_Z = 500;
//static const int MAX_Z = 9500;

int ActivityMap_Utils::MAX_X = 11000;
int ActivityMap_Utils::MIN_X = -11000;
int ActivityMap_Utils::MAX_Z = 9700;
int ActivityMap_Utils::MIN_Z = 500;


ActivityMap_Utils::ActivityMap_Utils(int ns):NUM_SENSORS(ns)
{	
	DEPTH_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  abs(MIN_X-MAX_X);

	XRes = ceil(double(YRes)*(double(X_VAR)/double(DEPTH_VAR)));

	depthStep = ceil(double(DEPTH_VAR)/double(YRes));
	xStep = ceil(double(X_VAR)/double(XRes));
}

ActivityMap_Utils::ActivityMap_Utils(void):NUM_SENSORS(3)
{
	DEPTH_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  abs(MIN_X-MAX_X);

	XRes = ceil(double(YRes)*(double(X_VAR)/double(DEPTH_VAR)));
}

ActivityMap_Utils::~ActivityMap_Utils(void)
{
}


void assignPointsToMat(XnPoint3D* realPoints, Mat& out)
{
	for (int r = 0; r < out.rows; r++)
	{
		out.ptr<float>(r)[0] = realPoints[r].X;
		out.ptr<float>(r)[1] = realPoints[r].Y;
		out.ptr<float>(r)[2] = realPoints[r].Z;
	}
}

void ActivityMap_Utils::createActivityMap(KinectSensor* kinects, const XnDepthPixel** depthMaps, const XnRGB24Pixel** rgbMaps, bool trans, Mat& activityMap, int nFrame)
{
	////create height path
	//char* path1 = "d:\\Emilio\\Tracking\\DataSet\\Heights\\height_";
	//char pathFull[100];
	//char nFrameStr[10];
	//itoa(nFrame, nFrameStr, 10);
	//strcpy(pathFull, path1);
	//strcat(pathFull, nFrameStr);
	//strcat(pathFull, ".yml");


	//Fits the noncalibrated map in an image
	int shifted = 0;
	if (!trans)
		shifted = 3400;

	//Mat* activityMap = new Mat(Size(XRes, YRes), CV_8UC3);
	Utils::initMat3u(activityMap, 255);
	Mat highestMap (YRes, XRes, CV_32F);
	Utils::initMatf(highestMap, -50000);
//	XnPoint3D** allRealPoints = new XnPoint3D*[NUM_SENSORS];

	Mat allRealPointsMat[3];

	int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES;
//	clock_t t = clock();
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		allRealPointsMat[i] = Mat(numPoints, 3, CV_32F);
		//create list of xnPoint3D
		XnPoint3D* depthPoints = convert2Points(depthMaps[i]); // allocates memory [Xn_vga_X_res*xn_vga_y_res]
		XnPoint3D* realPoints = kinects[i].arrayBackProject(depthPoints); //allocates memory [Xn_vga_X_res*xn_vga_y_res]
		if (trans)
			kinects[i].transformArray(realPoints, allRealPointsMat[i]);
		else
			assignPointsToMat(realPoints, allRealPointsMat[i]);

//		allRealPoints[i] = realPoints;
		delete[] depthPoints;
		delete[] realPoints;
	}

//	t = clock() - t;
//	cout << "It took " << t << " clicks (" << ((float)t)/CLOCKS_PER_SEC << " seconds).\n" << endl;

	
	//Go through all sensors
		XnPoint3D p;
		for (int iter = 0; iter < NUM_SENSORS; iter++)
		{
			//go through all points
			for (int i = 0; i < XN_VGA_Y_RES; i++)
			{
				for (int j = 0; j < XN_VGA_X_RES; j++)
				{
					float* allPtr = allRealPointsMat[iter].ptr<float>(i*XN_VGA_X_RES+j);
					p.X = allPtr[0];
					p.Y = allPtr[1];
					p.Z = allPtr[2];
					if (p.Z > 0 && p.Z < MAX_Z && p.Y > -400)// && p.Y < 0) // threshold the ceiling (hopefully)
					{
						int xCoor = findCoordinate((p.X-shifted), MIN_X, MAX_X, xStep);					
						int yC = findCoordinate(p.Z, MIN_Z, MAX_Z, depthStep);
						int yCoor = (YRes-1) - yC;

						float* ptr_h = highestMap.ptr<float>(yCoor);
				
						if (p.Y > ptr_h[xCoor])
						{
							ptr_h[xCoor] = p.Y;
							XnRGB24Pixel color = rgbMaps[iter][i*XN_VGA_X_RES+j];
			///TEST TO FIND OUT CAMERA POINTS
							//float errorRight = (-1.8427*xCoor + 1560) - yCoor;
							//float errorLeft = (1.8427*xCoor - 602) - yCoor;
							//if (errorLeft < 0)
							//{
							//	color.nBlue = 255; color.nGreen = 0; color.nRed = 0;
							//}
							//else if (errorRight < 0)
							//{
							//	color.nBlue = 0; color.nGreen = 0; color.nRed = 255;
							//}
							//else
							//{
							//	color.nBlue = 0; color.nGreen = 255; color.nRed = 0;
							//}
			//END TEST
							if (!trans)
							{
								circle(activityMap, Point((XRes/NUM_SENSORS)*iter+xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
							}
							else
							{
								uchar* ptr = activityMap.ptr<uchar>(yCoor);
								//circle(activityMap, Point(xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
								ptr[3*xCoor] = color.nBlue;
								ptr[3*xCoor+1] = color.nGreen;
								ptr[3*xCoor+2] = color.nRed;
							}
						}
					}
				}
			}

		}
	/*	FileStorage fs(pathFull, FileStorage::WRITE);
		fs << "HeightMap" << highestMap;
		fs.release();
		Point origin (activityMap.cols/2, activityMap.rows-1);
		Point dest (848,0);
		Point dest1(327,0);
		line(activityMap, origin, dest, Scalar(0,0,255));
		line(activityMap, origin, dest1, Scalar(0,0,255));*/

	//for (int iter = 0; iter < NUM_SENSORS; iter++)
	//{
	//	for (int i = 0; i < XN_VGA_Y_RES; i++)
	//	{
	//		for (int j = 0; j < XN_VGA_X_RES; j++)
	//		{
	//			XnPoint3D p = allRealPoints[iter][i*XN_VGA_X_RES+j];
	//			if (p.Z > 0 && p.Z < MAX_Z && p.Y < 0) // threshold the ceiling (hopefully)
	//			{
	//				int xCoor = findCoordinate((p.X-shifted), MIN_X, MAX_X, xStep);					
	//				int yC = findCoordinate(p.Z, MIN_Z, MAX_Z, depthStep);
	//				int yCoor = (YRes-1) - yC;

	//				float* ptr_h = highestMap.ptr<float>(yCoor);
	//			
	//				if (p.Y > ptr_h[xCoor])
	//				{
	//					ptr_h[xCoor] = p.Y;
	//					XnRGB24Pixel color = rgbMaps[iter][i*XN_VGA_X_RES+j];
	//					if (!trans)
	//					{
	//						circle(activityMap, Point((XRes/NUM_SENSORS)*iter+xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
	//					}
	//					else
	//						circle(activityMap, Point(xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
	//				}
	//			}
	//		}
	//	}
	//}
	//
	//for (int iter = 0; iter < NUM_SENSORS; iter++)
	//	delete[] allRealPoints[iter];
}


void ActivityMap_Utils::getImageBinned(const Mat* frame, Mat& binned, Size binSz, bool flag)
{
	for (int i = 0; i < binned.rows; i++)
	{
		ushort* ptrB = binned.ptr<ushort>(i);
		for (int j = 0; j < binned.cols; j++)
		{
			Rect roi = Rect(j*binSz.width, i*binSz.height, binSz.width, binSz.height);
			Mat bin = (*frame)(roi);
			int num = ((int)sum(bin).val[0])/255;
			ptrB[j] = num;
		}
	}
}


//void ActivityMap_Utils::getImageBinned(const Mat* frame, Mat& binned, Size binSz, bool flag)
//{
//	for (int i = 0; i < frame->rows; i++)
//	{
//		const uchar* imgPtr = frame->ptr<const uchar>(i);
//		for (int j = 0; j < frame->cols; j++)
//		{
//			//look for foreground points in the image
//			if ((flag && imgPtr[j*3+2] < 240) || (!flag && ((imgPtr[j*3]  < 240) || (imgPtr[j*3+1] < 240)  || (imgPtr[j*3+2] < 240) ))) 
//			{
//				//find the correct bin
//				int rowBin = (int)i/binSz.height;
//				int colBin = (int)j/binSz.width;
//				if (rowBin < 0 || rowBin >= binned.rows || colBin < 0 || colBin >= binned.cols)
//				{
//					if (rowBin == binned.rows)
//						rowBin--;
//					else if (colBin == binned.cols)
//						colBin--;
//					else
//						cout << "Error bin counting" << endl;
//
//				}
//				binned.ptr<ushort>(rowBin)[colBin] += 1;
//			}
//		}
//	}
//}


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