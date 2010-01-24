#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "Utils/wVector.h"
#include "Utils/wMatrix.h"

const int IMAGE_SEQ_COUNT = 200;
const char* IMAGE_SEQ_FILE_NAME = "seq/im%03d.pgm";

const double PROCESSING_TIME = 33.0;//ms

const int TEMPLATE_WIDTH = 100;
const int TEMPLATE_HEIGHT = 100;

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("sampling");
	cvNamedWindow("result");

	// initialize
	sprintf(message, IMAGE_SEQ_FILE_NAME, 0);
	IplImage* inputImage = cvLoadImage(message, 0);

	int width = inputImage->width;
	int height = inputImage->height;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	
	// set template image
	IplImage* templateImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* samplingImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);

	CvRect rect = cvRect(startX, startY, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	cvSetImageROI(inputImage, rect);
	cvCopyImage(inputImage, templateImage);
	cvShowImage("template", templateImage);

	std::vector<int> se;
	for(int y=0; y<templateImage->height; y++)
		for(int x=0; x<templateImage->width; x++)
			se.push_back(cvRound(cvGetReal2D(templateImage, y, x)));

	cvReleaseImage(&inputImage);
	IplImage* resultImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	// parameter
	windage::Matrix3 homography(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	homography._13 = startX;
	homography._23 = startY;

	bool processing =true;
	for(int i=0; i<IMAGE_SEQ_COUNT && processing; i++)
	{
		int64 startTime = cvGetTickCount();

		// load image
		sprintf(message, IMAGE_SEQ_FILE_NAME, i);
		IplImage* inputImage = cvLoadImage(message, 0);
		cvCvtColor(inputImage, resultImage, CV_GRAY2BGR);

		// processing
		// get 
		std::vector<int> sxc;
		for(int y=0; y<TEMPLATE_HEIGHT; y++)
		{
			for(int x=0; x<TEMPLATE_WIDTH; x++)
			{
				windage::Vector3 point(x, y, 1.0);
				windage::Vector3 out = homography * point;
				out /= out.z;

				double value = cvGetReal2D(inputImage, out.y, out.x);
				cvSetReal2D(samplingImage, y, x, value);
				sxc.push_back(cvRound(value));
			}
		}

		// update homography


		// draw result
		windage::Vector3 point1(0.0, 0.0, 1.0);
		windage::Vector3 point2(TEMPLATE_WIDTH, 0.0, 1.0);
		windage::Vector3 point3(TEMPLATE_WIDTH, TEMPLATE_HEIGHT, 1.0);
		windage::Vector3 point4(0.0, TEMPLATE_HEIGHT, 1.0);

		windage::Vector3 outPoint1 = homography * point1;
		windage::Vector3 outPoint2 = homography * point2;
		windage::Vector3 outPoint3 = homography * point3;
		windage::Vector3 outPoint4 = homography * point4;

		outPoint1 /= outPoint1.z;
		outPoint2 /= outPoint2.z;
		outPoint3 /= outPoint3.z;
		outPoint4 /= outPoint4.z;

		cvLine(resultImage, cvPoint(outPoint1.x, outPoint1.y), cvPoint(outPoint2.x, outPoint2.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint2.x, outPoint2.y), cvPoint(outPoint3.x, outPoint3.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint3.x, outPoint3.y), cvPoint(outPoint4.x, outPoint4.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint4.x, outPoint4.y), cvPoint(outPoint1.x, outPoint1.y), CV_RGB(255, 0, 0));

		// draw image
		cvShowImage("sampling", samplingImage);
		cvShowImage("result", resultImage);
		cvReleaseImage(&inputImage);

		int64 endTime = cvGetTickCount();
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << i << " : processing time : " << processingTime << " ms" << std::endl;

		int waittingTime = cvRound(PROCESSING_TIME - processingTime);
		if(waittingTime < 1) waittingTime = 1;
//		waittingTime = 0;
		char ch = cvWaitKey(waittingTime);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	cvReleaseImage(&resultImage);
	cvDestroyAllWindows();
}