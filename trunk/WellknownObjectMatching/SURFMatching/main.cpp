#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "Logger.h"

using namespace std;
const int WIDTH = 640;
const int HEIGHT = 480;

const int EXTENTION = 1;
const int DESCRIPTOR_DIMENSION = 128;
const double THRESHOLD = 1000.0;

typedef struct _SURFFeaturePoint
{
	CvPoint2D32f point;
	int size;
	float descriptor[DESCRIPTOR_DIMENSION];

	void operator=(struct _SURFFeaturePoint oprd)
	{
		point = oprd.point;
		size = oprd.size;
		for(int i=0; i<DESCRIPTOR_DIMENSION; i++)
			descriptor[i] = oprd.descriptor[i];
	}
}SURFFeaturePoint;

CvFeatureTree* CreateFeatureTree(std::vector<SURFFeaturePoint>* points, CvMat* featureTreeStorage)
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

void ExtractSURFDescriptor(IplImage* image, std::vector<SURFFeaturePoint>* points)
{
	CvSeq* keypoints;
	CvSeq* descriptors;
	CvMemStorage* storage = cvCreateMemStorage(0);

	CvSURFParams params = cvSURFParams(THRESHOLD, EXTENTION);
	cvExtractSURF(image, 0, &keypoints, &descriptors, storage, params);

	CvSeqReader reader;
	cvStartReadSeq(descriptors, &reader, 0);

	SURFFeaturePoint point;
	for(int i=0; i<keypoints->total; i++)
	{
		CvSURFPoint* pt1 = (CvSURFPoint*)cvGetSeqElem(keypoints, i);
		point.point = pt1->pt;
		point.size = pt1->size;

		const float* vec = (const float*)reader.ptr;
		CV_NEXT_SEQ_ELEM(reader.seq->elem_size, reader);

		for(int j=0; j<DESCRIPTOR_DIMENSION; j++)
		{
			point.descriptor[j] = vec[j];
		}

		points->push_back(point);
	}

	cvReleaseMemStorage(&storage);
}

Logger* tempLog = new Logger("ORIGINAL_SURF_matchiing", true);
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

int FindPairs(SURFFeaturePoint description, CvFeatureTree* tree, double distanceRate = 0.7)
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
	Logger* log = new Logger("ORIGINAL_SURF", true);

	std::vector<SURFFeaturePoint> referencePoints;

	IplImage* referenceImage = cvLoadImage("reference_map.png", 0);
	IplImage* referenceColor = cvLoadImage("reference_map.png");
	CvFeatureTree* referenceTree = NULL;
	CvMat* referenceTreeStorage = NULL;

	ExtractSURFDescriptor(referenceImage, &referencePoints);
	referenceTree = CreateFeatureTree(&referencePoints, referenceTreeStorage);

	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT*2), IPL_DEPTH_8U, 3);
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	const char* fileformat = "d:\\ImageSequence\\20090912-2\\capture%d.jpg";
//	const char* fileformat = "testset\\degree_%d.png";
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

		std::vector<SURFFeaturePoint> points;

		std::vector<CvPoint2D32f> matchedReferencePoints;
		std::vector<CvPoint2D32f> matchedImagePoints;

		log->updateTickCount();
		ExtractSURFDescriptor(grayImage, &points);
		log->log("ExtractSURF", log->calculateProcessTime());

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
				int size = referencePoints[index].size/2.0;
				cvRectangle(resultImage, cvPoint(referencePoints[index].point.x-size/2, referencePoints[index].point.y-size/2), cvPoint(referencePoints[index].point.x+size/2, referencePoints[index].point.y+size/2), CV_RGB(0, 255, 255), 2);

				size = points[j].size/2.0;
				cvRectangle(resultImage, cvPoint(points[j].point.x-size/2, HEIGHT + points[j].point.y-size/2), cvPoint(points[j].point.x+size/2, HEIGHT + points[j].point.y+size/2), CV_RGB(255, 255, 0), 2);

				cvLine(resultImage, cvPoint(referencePoints[index].point.x, referencePoints[index].point.y), cvPoint(points[j].point.x, HEIGHT+points[j].point.y), CV_RGB(0, 255, 0));
			}			
		}

		cvShowImage("tracking", inputImage);
		cvShowImage("result", resultImage);

		char ch;
		if(isBreak) ch = cvWaitKey();
		else ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			isBreak = !isBreak;
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
			sprintf(filename, "SURF_savesample%d_1.png", i);
			cvSaveImage(filename, inputImage);
			sprintf(filename, "SURF_savesample%d_2.png", i);
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