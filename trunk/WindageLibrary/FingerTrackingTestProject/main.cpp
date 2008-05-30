
#include <cv.h>
#include <highgui.h>

#include <vector>
#include <iostream>
using namespace std;

#include "../include/windageFingerTracking.h"

#define M_PI 3.14159265358979323846 

void main()
{
	IplImage* image = cvLoadImage("../Data/handimage2.jpg");
	IplImage* result = cvCloneImage(image);

	IplImage* skinDetectionResult = cvCreateImage(cvGetSize(image), image->depth, image->nChannels);
	SkinDetection(skinDetectionResult, image);

	IplImage* handDetectionResult = cvCreateImage(cvGetSize(image), image->depth, image->nChannels);
	Vector3 centerPosition;
	HandDetection(handDetectionResult, &centerPosition, skinDetectionResult);

	double radius = sqrt(centerPosition.z / M_PI) * 1.1f;
	cvCircle(result, cvPoint((int)centerPosition.x, (int)centerPosition.z), (int)radius, CV_RGB(0, 255, 255));

	vector<Vector3> convexPositiontList;
	ConvexHull(&convexPositiontList, handDetectionResult, radius * 0.2f, result);

	vector<Vector3> fingerPosiontList;
	int border = 10;
	FindFingerTipPoint(&fingerPosiontList, &convexPositiontList, centerPosition, radius, border, handDetectionResult, result);

	int i;
	for(i=0; i<(int)fingerPosiontList.size(); i++)
	{
		cvLine(result, cvPoint((int)fingerPosiontList[i].x, (int)fingerPosiontList[i].y),
						cvPoint((int)centerPosition.x, (int)centerPosition.y), CV_RGB(0, 255, 0));
		cvCircle(result, cvPoint((int)fingerPosiontList[i].x, (int)fingerPosiontList[i].y), (int)(radius*0.2f), CV_RGB(255, 255, 0));
	}

	cvNamedWindow("reulst");
	cvShowImage("reulst", result);

	cvWaitKey();

	cvReleaseImage(&handDetectionResult);
	cvReleaseImage(&skinDetectionResult);
	cvReleaseImage(&result);
	cvReleaseImage(&image);
}