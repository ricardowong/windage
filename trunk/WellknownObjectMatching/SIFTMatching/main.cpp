#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "sift.h"
#include "imgfeatures.h"

#include "Logger.h"

using namespace std;
const int WIDTH = 640;
const int HEIGHT = 480;

const int EXTENTION = 1;
const int DESCRIPTOR_DIMENSION = 128;
const double THRESHOLD = 1000.0;

typedef struct _SIFTFeaturePoint
{
	CvPoint2D32f point;
	int size;
	float descriptor[DESCRIPTOR_DIMENSION];

	void operator=(struct _SIFTFeaturePoint oprd)
	{
		point = oprd.point;
		size = oprd.size;
		for(int i=0; i<DESCRIPTOR_DIMENSION; i++)
			descriptor[i] = oprd.descriptor[i];
	}
}SIFTFeaturePoint;

CvFeatureTree* CreateFeatureTree(std::vector<SIFTFeaturePoint>* points, CvMat* featureTreeStorage)
{
	int count = points->size();
	featureTreeStorage = cvCreateMat(count, DESCRIPTOR_DIMENSION, CV_32FC1);

	int length = DESCRIPTOR_DIMENSION;
	for(int y=0; y<count; y++)
	{
		for(int x=0; x<DESCRIPTOR_DIMENSION; x++)
		{
			cvSetReal2D(featureTreeStorage, y, x, (*points)[y].descriptor[x]);
		}
	}

	CvFeatureTree* tree = cvCreateFeatureTree(featureTreeStorage);
	return tree;
}

void ExtractSIFTDescriptor(IplImage* image, std::vector<SIFTFeaturePoint>* points)
{
	struct feature* feature;

	int count = sift_features( image, &feature);

	for(int i=0; i<count; i++)
	{
		SIFTFeaturePoint featurePoint;
		featurePoint.point.x = feature[i].x;
		featurePoint.point.y = feature[i].y;
		featurePoint.size = feature[i].scl;
		for(int j=0; j<DESCRIPTOR_DIMENSION; j++)
		{
			featurePoint.descriptor[j] = feature[i].descr[j];
		}
		points->push_back(featurePoint);
	}

	for(int i=0; i<count; i++)
	{
		delete &(feature[i]);
	}
	delete feature;
}

Logger* tempLog = new Logger("ORIGINAL_SIFT_matchiing", true);
double CalculatePose(std::vector<CvPoint2D32f>* matchedReferencePoints, std::vector<CvPoint2D32f>* matchedScenePoints)
{
	float homography[9];
	const double ERROR_BOUND = 2.0;

	float homographyError = 0;
	int n = (int)(*matchedReferencePoints).size();
	if(n >= 4)
	{
		int inlierCount = 0;
		int outlierCount = 0;

		CvMat _h = cvMat(3, 3, CV_32F, homography);

		CvMat _pt1 = cvMat(1, n, CV_32FC2, &((*matchedReferencePoints)[0]) );
		CvMat _pt2 = cvMat(1, n, CV_32FC2, &((*matchedScenePoints)[0]) );

		homographyError = 0.0;
		if(cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, ERROR_BOUND))
		{
			// calculate homography error & remove outlier
			float difference = 0;
			int count = 0;
			for(unsigned int i=0; i<(*matchedReferencePoints).size(); i++)
			{

				float point[3];
				point[0] = (*matchedReferencePoints)[i].x;
				point[1] = (*matchedReferencePoints)[i].y;
				point[2] = 1.0;

				float projectionPointX = homography[0] * point[0] + homography[1] * point[1] + homography[2] * point[2];
				float projectionPointY = homography[3] * point[0] + homography[4] * point[1] + homography[5] * point[2];
				float projectionPointZ = homography[6] * point[0] + homography[7] * point[1] + homography[8] * point[2];
				projectionPointX /= projectionPointZ;
				projectionPointY /= projectionPointZ;

				std::vector<CvPoint2D32f>::iterator it1 = (*matchedReferencePoints).begin();
				std::vector<CvPoint2D32f>::iterator it2 = (*matchedScenePoints).begin();

				float dx = abs((*matchedScenePoints)[i].x - projectionPointX);
				float dy = abs((*matchedScenePoints)[i].y - projectionPointY);
				if(dx + dy <= ERROR_BOUND)
				{
//					difference += abs((*matchedScenePoints)[i].x - projectionPointX);
//					difference += abs((*matchedScenePoints)[i].y - projectionPointY);
					count++;

					inlierCount++;
				}
				else // outlier
				{
//					(*matchedReferencePoints).erase((*matchedReferencePoints).begin() + count);
//					(*matchedScenePoints).erase((*matchedScenePoints).begin() + count);

					outlierCount++;
				}

				difference += (dx + dy);
			}
			homographyError = (difference / 2.) / (float)(*matchedReferencePoints).size();
		}

		if(tempLog) tempLog->log("inliner", inlierCount);
		if(tempLog) tempLog->log("outliner", outlierCount);
		if(tempLog) tempLog->logNewLine();
	}
	else
	{
		homographyError = -1.0;
	}

	return homographyError;
}

int FindPairs(SIFTFeaturePoint description, CvFeatureTree* tree, double distanceRate = 0.7)
{
	CvMat* currentFeature = cvCreateMat(1, DESCRIPTOR_DIMENSION, CV_32FC1);
	CvMat* result = cvCreateMat(1, 2, CV_32SC1);
	CvMat* distance = cvCreateMat(1, 2, CV_64FC1);
	for(int x=0; x<DESCRIPTOR_DIMENSION; x++)
	{
		cvSetReal2D(currentFeature, 0, x, description.descriptor[x]);
	}

	cvFindFeatures(tree, currentFeature, result, distance, 2, 20);

	double min2 = cvGetReal2D(distance, 0, 0);
	double min1 = cvGetReal2D(distance, 0, 1);
	int index2 = (int)cvGetReal2D(result, 0, 0);
	int index1 = (int)cvGetReal2D(result, 0, 1);
	int index = index1;
//*
	double temp;
	if(min2 < min1)
	{
		temp = min1;
		min1 = min2;
		min2 = temp;
		index = index2;
	}
//*/
	cvReleaseMat(&currentFeature);
	cvReleaseMat(&result);
	cvReleaseMat(&distance);

	if ( min1 < distanceRate*min2 )
		return index;
    return -1;
}

void main()
{
	Logger* log = new Logger("ORIGINAL_SIFT", true);

	std::vector<SIFTFeaturePoint> referencePoints;

	IplImage* referenceImage = cvLoadImage("reference_map.png", 0);
	IplImage* referenceColor = cvLoadImage("reference_map.png");
	CvFeatureTree* referenceTree = NULL;
	CvMat* referenceTreeStorage = NULL;

	ExtractSIFTDescriptor(referenceImage, &referencePoints);
	referenceTree = CreateFeatureTree(&referencePoints, referenceTreeStorage);

	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT*2), IPL_DEPTH_8U, 3);
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	const char* fileformat = "d:\\ImageSequence\\20090912-2\\capture%d.jpg";
//	const char* fileformat = "testset\\rotation_%d.png";
	const int step = 1;

	char filename[100];
	int index = 0;

	cvNamedWindow("tracking");
	cvNamedWindow("result");

	bool isBreak = true;
	bool processing = true;
	for(int i=0; i<4736&&processing; i+=step)
	{
		sprintf(filename, fileformat, i);
		IplImage* grabFrame = cvLoadImage(filename);
		cvCopy(grabFrame, inputImage);
		cvReleaseImage(&grabFrame);

		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		
		cvSetImageROI(resultImage, cvRect(0, 0, WIDTH, HEIGHT));
		cvCopyImage(referenceColor, resultImage);
		cvSetImageROI(resultImage, cvRect(0, HEIGHT, WIDTH, HEIGHT));
		cvCopyImage(inputImage, resultImage);
		cvResetImageROI(resultImage);

		std::vector<SIFTFeaturePoint> points;

		std::vector<CvPoint2D32f> matchedReferencePoints;
		std::vector<CvPoint2D32f> matchedImagePoints;

		log->updateTickCount();
		ExtractSIFTDescriptor(grayImage, &points);
		log->log("ExtractSIFT", log->calculateProcessTime());

		log->updateTickCount();
		for(int j=0; j<points.size(); j++)
		{
			int index = FindPairs(points[j], referenceTree);
			if(index > 0)
			{
				matchedImagePoints.push_back(points[j].point);
				matchedReferencePoints.push_back(referencePoints[index].point);
			}
		}
		log->log("Matching", log->calculateProcessTime());

		log->updateTickCount();
		CalculatePose(&matchedReferencePoints, &matchedImagePoints);
		log->log("PoseEstimation", log->calculateProcessTime());

		log->logNewLine();

		cvResetImageROI(resultImage);
		for(int j=0; j<points.size(); j++)
		{
			int index = FindPairs(points[j], referenceTree);
			if(index > 0)
			{
				cvCircle(resultImage, cvPoint(referencePoints[index].point.x, referencePoints[index].point.y),  referencePoints[index].size, CV_RGB(0, 255, 255), 2);
				cvCircle(resultImage, cvPoint(points[j].point.x, HEIGHT + points[j].point.y),					points[j].size,				 CV_RGB(255, 255, 0), 2);

				cvLine(resultImage, cvPoint(referencePoints[index].point.x, referencePoints[index].point.y), cvPoint(points[j].point.x, HEIGHT+points[j].point.y), CV_RGB(0, 0, 255));
			}			
		}

		cvShowImage("tracking", inputImage);
		cvShowImage("result", resultImage);

		std::cout << i << std::endl;

		char ch;
		if(isBreak) ch = cvWaitKey();
		else ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			isBreak = !isBreak;
			break;
		case '[':
		case '{':
			i -= 10;
			break;
		case ']':
		case '}':
			i += 10;
			break;
		case '.':
		case '>':
			i += 100;
			break;
		case ',':
		case '<':
			i -= 100;
			break;
		case 's':
		case 'S':
			sprintf(filename, "SIFT_savesample%d_1.png", i);
			cvSaveImage(filename, inputImage);
			sprintf(filename, "SIFT_savesample%d_2.png", i);
			cvSaveImage(filename, resultImage);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	cvDestroyAllWindows();
}