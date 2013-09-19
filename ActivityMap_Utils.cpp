#include "ActivityMap_Utils.h"


//values for dataset lab all
//static const int MIN_X = -11000;
//static const int MAX_X = 11000;
//static const int MIN_Z = 500;
//static const int MAX_Z = 9500;

int ActivityMap_Utils::MAX_X = 11000;
int ActivityMap_Utils::MIN_X = -11000;
int ActivityMap_Utils::MAX_Z = 9700;
int ActivityMap_Utils::MIN_Z = 0;
int ActivityMap_Utils::MAX_Z_TRANS = 9700;

//INITIALIZE VALUES
int ActivityMap_Utils::Z_STEP = 20;
int ActivityMap_Utils::X_STEP = 20;
int ActivityMap_Utils::Z_VAR = 9200;
int ActivityMap_Utils::X_VAR = 22000;
int ActivityMap_Utils::X_RES = 1148;
int ActivityMap_Utils::Y_RES = 480;

float ActivityMap_Utils::RATIO = 0.418;
int ActivityMap_Utils::CEILING_THRESHOLD = -400;
int ActivityMap_Utils::FLOOR_THRESHOLD = -2400;


ActivityMap_Utils::ActivityMap_Utils(float scale, int ns):NUM_SENSORS(ns)
{	
	MAX_Z = scale*MAX_Z;

	MAX_Z_TRANS = MAX_Z/(cosf(KinectSensor::KINECT_HORIZ_FOV/2));

	Z_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  Z_VAR/RATIO;
	MIN_X = -X_VAR/2;
	MAX_X = -MIN_X;

	Y_RES = scale*Y_RES;

	X_RES = ceil(double(Y_RES)*(double(X_VAR)/double(Z_VAR)));

	Z_STEP = ceil(double(Z_VAR)/double(Y_RES));
	X_STEP = ceil(double(X_VAR)/double(X_RES));
}

ActivityMap_Utils::ActivityMap_Utils(void):NUM_SENSORS(3)
{
	Z_VAR = abs(MIN_Z-MAX_Z);
	X_VAR =  abs(MIN_X-MAX_X);

	X_RES = ceil(double(Y_RES)*(double(X_VAR)/double(Z_VAR)));
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


Point ActivityMap_Utils::findMoACoordinate(const XnPoint3D* p, int threshRange)
{
	float range = sqrtf(pow(p->X,2) + pow(p->Z,2));
	Point out = Point(-1,-1);
	if (p->X > MIN_X && p->X < MAX_X && p->Z < MAX_Z_TRANS && p->Y < CEILING_THRESHOLD && p->Y > FLOOR_THRESHOLD && range < threshRange)
	{
		out.x = floor((p->X-MIN_X)/X_STEP);
		out.y = (Y_RES - 1) - floor((p->Z/Z_STEP));
		//out.y = (Y_RES - 1) - floor(((p->Z-MIN_Z)/Z_STEP));
	}
	return out;
}


void ActivityMap_Utils::createActivityMap(KinectSensor* kinects, const XnDepthPixel** depthMaps, const XnRGB24Pixel** rgbMaps, bool trans, Mat& activityMap, int nFrame, int thresh, float* timeIntervals)
{
	clock_t startTime_tmp;
	//Fits the noncalibrated map in an image
	int shifted = 0;
	if (!trans)
		shifted = 3400;

	//Mat* activityMap = new Mat(Size(XRes, YRes), CV_8UC3);
	Utils::initMat3u(activityMap, 255);
	Mat highestMap (Y_RES, X_RES, CV_32F);
	Utils::initMatf(highestMap, -50000);
//	XnPoint3D** allRealPoints = new XnPoint3D*[NUM_SENSORS];

	Mat allRealPointsMat[3];

	int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES;
//	clock_t t = clock();
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		allRealPointsMat[i] = Mat(numPoints, 3, CV_32F);
		//create list of xnPoint3D
		startTime_tmp = clock();
		XnPoint3D* depthPoints = convert2Points(depthMaps[i]); // allocates memory [Xn_vga_X_res*xn_vga_y_res]
		timeIntervals[0] += clock() - startTime_tmp; //time debugging

		startTime_tmp = clock();
		XnPoint3D* realPoints = kinects[i].arrayBackProject(depthPoints); //allocates memory [Xn_vga_X_res*xn_vga_y_res]
		timeIntervals[1] += clock() - startTime_tmp; //time debugging

		if (trans)
		{
			startTime_tmp = clock();
			kinects[i].transformArray(realPoints, allRealPointsMat[i]);
			timeIntervals[2] += clock() - startTime_tmp; //time debugging
		}
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

					startTime_tmp = clock();
					//Point p2D = findMoACoordinate(&p, thresh);
					int X = p.X;
					int Y = p.Y;
					int Z = p.Z;
					float range = sqrtf(powf(X,2) + powf(Z,2));
					Point p2D = Point(-1,-1);
					if (X > MIN_X && X < MAX_X && Z < MAX_Z_TRANS && Y < CEILING_THRESHOLD && Y > FLOOR_THRESHOLD && range < thresh)
					{
						p2D.x = floorf((X-MIN_X)/X_STEP);
						p2D.y = (Y_RES - 1) - floorf((Z/Z_STEP));
					}
					timeIntervals[3] += clock() - startTime_tmp; //time debugging

					if (p2D.x != -1 && p2D.y != -1)
					{
						float* ptr_h = highestMap.ptr<float>(p2D.y);
				
						if (p.Y > ptr_h[p2D.x])
						{
							ptr_h[p2D.x] = p.Y;
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
								uchar* ptr = activityMap.ptr<uchar>(p2D.y);
								//circle(activityMap, Point((XRes/NUM_SENSORS)*iter+xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
								int xCoor = (X_RES/NUM_SENSORS)*iter+p2D.x;
								ptr[3*xCoor] = color.nBlue;
								ptr[3*xCoor+1] = color.nGreen;
								ptr[3*xCoor+2] = color.nRed;
							}
							else
							{
								uchar* ptr = activityMap.ptr<uchar>(p2D.y);
								//circle(activityMap, Point(xCoor, yCoor), 1, cvScalar(color.nBlue,color.nGreen, color.nRed), 2);
								ptr[3*p2D.x] = color.nBlue;
								ptr[3*p2D.x+1] = color.nGreen;
								ptr[3*p2D.x+2] = color.nRed;
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