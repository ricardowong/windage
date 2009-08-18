#include "Segmentation.h"

LaserSegmentation::LaserSegmentation()
{
	initialized = false;
	segmentedImage = NULL;
}

LaserSegmentation::~LaserSegmentation()
{
	this->Release();
}

void LaserSegmentation::Release()
{
	if(segmentedImage) cvReleaseImage(&segmentedImage);
	segmentedImage = NULL;
	initialized = false;
}

bool LaserSegmentation::Initialize(IplImage* tempImage)
{
	this->Release();
	segmentedImage = cvCreateImage(cvGetSize(tempImage), IPL_DEPTH_8U, 1);
	initialized = true;
	return true;
}

/*
IplImage* LaserSegmentation::Segment(IplImage* inputImage, double threshold)
{
	#pragma omp parallel for
	for(int y=0; y<inputImage->height; y++)
	{
		for(int x=0; x<inputImage->width; x++)
		{
			CvScalar color = cvGet2D(inputImage, y, x);

			double brightness = (299 * color.val[2] + 587 * color.val[1] + 114 * color.val[0]) / 1000;
			if(brightness > threshold)
			{
				color.val[0] = color.val[1] = color.val[2] = color.val[2];
			}
			else
			{
				color.val[0] = color.val[1] = color.val[2] = 0;
			}

			cvSet2D(segmentedImage, y, x, color);
		}
	}

	return segmentedImage;
}
//*/
//*
IplImage* LaserSegmentation::Segment(IplImage* inputImage, double threshold)
{
	#pragma omp parallel for
	for(int y=0; y<inputImage->height; y++)
	{
		for(int x=0; x<inputImage->width; x++)
		{
			CvScalar color = cvGet2D(inputImage, y, x);

			if(200 < color.val[2] && color.val[2] < 256 &&
				00 < color.val[1] && color.val[1] < 150 &&
				00 < color.val[0] && color.val[0] < 150 )
			{
				if(color.val[2] > color.val[1] * 2 && color.val[2] > color.val[0] * 2)
					color.val[0] = color.val[1] = color.val[2] = color.val[2];
				else
					color.val[0] = color.val[1] = color.val[2] = 0;
			}
			else
			{
				color.val[0] = color.val[1] = color.val[2] = 0;
			}

			cvSet2D(segmentedImage, y, x, color);
		}
	}

	return segmentedImage;
}
//*/

IplImage* LaserSegmentation::FillHole()
{
	cvErode(segmentedImage, segmentedImage);
	cvErode(segmentedImage, segmentedImage);
	cvDilate(segmentedImage, segmentedImage);
	cvDilate(segmentedImage, segmentedImage);

	cvDilate(segmentedImage, segmentedImage);
	cvDilate(segmentedImage, segmentedImage);
	cvErode(segmentedImage, segmentedImage);
	cvErode(segmentedImage, segmentedImage);

	return segmentedImage;
}

CvPoint LaserSegmentation::GetMaxContour(int sizeThreshold)
{
	CvPoint centerPoint = cvPoint(0, 0);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours = 0;
    int i, j, comp_count = 0;

	IplImage* contourImage = cvCloneImage(segmentedImage);
	cvFindContours(contourImage, storage, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	int maxSize = 0;
	CvPoint maxPoint = cvPoint(0, 0);
	CvSeq* maxContours = NULL;
	for(; contours != 0; contours = contours->h_next)
    {
		int size = contours->total;
		if(maxSize < size)
		{
			maxContours = contours;
			maxSize = size;
		}
    }

	if(maxSize > sizeThreshold)
	{
		cvZero(segmentedImage);
		cvDrawContours(segmentedImage, maxContours, cvScalar(255, 255, 255), cvScalar(255, 255, 255), -1, -1, 8);

		int count = 0;
		for(int y=0; y<segmentedImage->height; y++)
		{
			for(int x=0; x<segmentedImage->width; x++)
			{
				if(cvGetReal2D(segmentedImage, y, x) > 0)
				{
					centerPoint.x += x;
					centerPoint.y += y;
					count++;
				}
			}
		}
		centerPoint.x /= count;
		centerPoint.y /= count;
	}

	cvReleaseImage(&contourImage);
	cvReleaseMemStorage( &storage );

	return centerPoint;
}