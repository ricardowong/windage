#include "OpticalFlow.h"
using namespace windage;

OpticalFlow::OpticalFlow()
{
	terminationCriteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);
	prevImage = NULL;
	pyramid1 = NULL;
	pyramid2 = NULL;
}

OpticalFlow::~OpticalFlow()
{
	this->Release();
}

void OpticalFlow::Release()
{
	if(prevImage) cvReleaseImage(&prevImage);
	prevImage = NULL;
	if(pyramid1) cvReleaseImage(&pyramid1);
	pyramid1 = NULL;
	if(pyramid2) cvReleaseImage(&pyramid2);
	pyramid2 = NULL;
}

void OpticalFlow::Initialize(int width, int height, CvSize windowSize, int pyramidLevel)
{
	this->Release();
	this->SetImageSize(width, height);

	this->SetWindowSize(windowSize);
	this->SetPyramidLevel(pyramidLevel);

	terminationCriteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);
	prevImage = cvCreateImage(this->GetImageSize(), IPL_DEPTH_8U, 1);
	pyramid1 = cvCreateImage(this->GetImageSize(), IPL_DEPTH_8U, 1);
	pyramid2 = cvCreateImage(this->GetImageSize(), IPL_DEPTH_8U, 1);

	cvZero(prevImage);
}

int OpticalFlow::TrackFeature(IplImage* grayImage, std::vector<CvPoint2D32f>* prevPoints, std::vector<CvPoint2D32f>* currPoints)
{
	int pointCount = MIN((int)prevPoints->size(), this->maxPointCount);
	if(pointCount >= 1)
	{
		for(int i=0; i<pointCount; i++)
			this->feature1[i] = (*prevPoints)[i];

		cvCalcOpticalFlowPyrLK(prevImage, grayImage, pyramid1, pyramid2, feature1, feature2, pointCount, this->windowSize, this->pyramidLevel, foundFeature, errorFeature, terminationCriteria, 0);

		int index = 0;
		for(int i=0; i<pointCount; i++)
		{
			if(foundFeature[i] != 0)
			{
				currPoints->push_back(feature2[i]);
			}
			else
			{
				currPoints->push_back(cvPoint2D32f(-1.0, -1.0));
			}
		}

		cvCopyImage(grayImage, prevImage);
		return 1;
	}
	else
	{
		cvCopyImage(grayImage, prevImage);
		return 0;
	}
}

