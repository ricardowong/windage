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

const double DELTA = 1.0;
const int HOMOGRAPHY_COUNT = 8;

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("sampling");
	cvNamedWindow("result");

	// initialize
	sprintf(message, IMAGE_SEQ_FILE_NAME, 0);
	IplImage* saveImage = cvLoadImage(message, 0);

	int width = saveImage->width;
	int height = saveImage->height;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	double q = TEMPLATE_WIDTH * TEMPLATE_HEIGHT;
	
	// set template image
	IplImage* templateImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* samplingImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);

	CvMat* Je = cvCreateMat(q, HOMOGRAPHY_COUNT, CV_64F);
	CvMat* Jx = cvCreateMat(q, HOMOGRAPHY_COUNT, CV_64F);
	CvMat* Jsum = cvCreateMat(q, HOMOGRAPHY_COUNT, CV_64F);
	CvMat* JsumT = cvCreateMat(HOMOGRAPHY_COUNT, q, CV_64F);
	CvMat* J = cvCreateMat(HOMOGRAPHY_COUNT, HOMOGRAPHY_COUNT, CV_64F);
	CvMat* Jinvers = cvCreateMat(HOMOGRAPHY_COUNT, HOMOGRAPHY_COUNT, CV_64F);
	CvMat* ds = cvCreateMat(q, 1, CV_64F);
	CvMat* JTds = cvCreateMat(HOMOGRAPHY_COUNT, 1, CV_64F);
	CvMat* dx = cvCreateMat(HOMOGRAPHY_COUNT, 1, CV_64F);

	CvRect rect = cvRect(startX, startY, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	cvSetImageROI(saveImage, rect);
	cvCopyImage(saveImage, templateImage);
	cvShowImage("template", templateImage);
	cvResetImageROI(saveImage);

	IplImage* resultImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	// initial homography
	windage::Matrix3 homography(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	homography._13 = startX;
	homography._23 = startY;
	windage::Matrix3 e = homography;

	bool processing =true;
	for(int i=0; i<IMAGE_SEQ_COUNT && processing; i++)
	{
		int64 startTime = cvGetTickCount();

		// load image
		sprintf(message, IMAGE_SEQ_FILE_NAME, i);
		IplImage* inputImage = cvLoadImage(message, 0);
		cvCvtColor(inputImage, resultImage, CV_GRAY2BGR);

		// processing
		// sampling the current image

		// se
		std::vector<double> se;
		for(int y=0; y<templateImage->height; y++)
		{
			for(int x=0; x<templateImage->width; x++)
			{
				double value = cvGetReal2D(templateImage, y, x);
				se.push_back(value);
			}
		}

		// sxc
		std::vector<double> sxc;
		for(int y=0; y<TEMPLATE_HEIGHT; y++)
		{
			for(int x=0; x<TEMPLATE_WIDTH; x++)
			{
				windage::Vector3 point(x, y, 1.0);
				windage::Vector3 out = homography * point;
				out /= out.z;

				double value = -1.0;
				if( 0 < out.x && out.x < inputImage->width &&
					0 < out.y && out.y < inputImage->height)
					value = cvGetReal2D(inputImage, out.y, out.x);
				cvSetReal2D(samplingImage, y, x, value);
				sxc.push_back(value);
			}
		}

		// update homography
		windage::Vector3 point1, point2, out1, out2;
		point1.z = point2.z = 1.0;
		for(int y=0; y<TEMPLATE_HEIGHT; y++)
		{
			for(int x=0; x<TEMPLATE_WIDTH; x++)
			{
				double I1 = -1.0;
				double I2 = -1.0;

				// dI(p) / dp
				windage::Vector2 dI;
				point1.x = x - DELTA;
				point1.y = y;
				point2.x = x + DELTA;
				point2.y = y;
				out1 = e * point1;
				out2 = e * point2;
				out1 /= out1.z;
				out2 /= out2.z;

				I1 = cvGetReal2D(saveImage, out1.y, out1.x);
				I2 = cvGetReal2D(saveImage, out2.y, out2.x);
				dI.x = (I2 - I1)/(2*DELTA);

				point1.x = x;
				point1.y = y - DELTA;
				point2.x = x;
				point2.y = y + DELTA;
				out1 = e * point1;
				out2 = e * point2;
				out1 /= out1.z;
				out2 /= out2.z;

				I1 = cvGetReal2D(saveImage, out1.y, out1.x);
				I2 = cvGetReal2D(saveImage, out2.y, out2.x);
				dI.y = (I2 - I1)/(2*DELTA);

				// dIw(p) / dp
				windage::Vector2 dwI;
				point1.x = x - DELTA;
				point1.y = y;
				point2.x = x + DELTA;
				point2.y = y;
				out1 = homography * point1;
				out2 = homography * point2;
				out1 /= out1.z;
				out2 /= out2.z;

				I1 = cvGetReal2D(saveImage, out1.y, out1.x);
				I2 = cvGetReal2D(saveImage, out2.y, out2.x);
				dwI.x = (I2 - I1)/(2*DELTA);

				point1.x = x;
				point1.y = y - DELTA;
				point2.x = x;
				point2.y = y + DELTA;
				out1 = homography * point1;
				out2 = homography * point2;
				out1 /= out1.z;
				out2 /= out2.z;

				I1 = cvGetReal2D(saveImage, out1.y, out1.x);
				I2 = cvGetReal2D(saveImage, out2.y, out2.x);
				dwI.y = (I2 - I1)/(2*DELTA);

				// dw(x) / dx (2xp jacobian matrix)
				std::vector<windage::Vector2> dwx;
				point1.x = x;
				point1.y = y;

				for(int i=0; i<HOMOGRAPHY_COUNT; i++)
				{
					windage::Matrix3 tempHomography1 = e;
					windage::Matrix3 tempHomography2 = e;

					tempHomography1.m1[i] -= DELTA;
					tempHomography2.m1[i] += DELTA;

					out1 = tempHomography1 * point1;
					out2 = tempHomography2 * point1;
//					out1 /= out1.z;
//					out2 /= out2.z;

					out1 = (out2 - out1)/(2*DELTA);
//					out1 /= out1.z;
					windage::Vector2 temp(out1.x, out1.y);
					dwx.push_back(temp);
				}

				// Jsum = J(e) + J(xc)
				for(int i=0; i<HOMOGRAPHY_COUNT; i++)
				{
					double value = (dI + dwI) * dwx[i];
					cvSetReal2D(Jsum, y*TEMPLATE_WIDTH + x, i, value);
				}
			}
		}

		// delta_s
		for(int i=0; i<q; i++)
		{
			double value = (sxc[i] - se[i]);
			cvSetReal2D(ds, i, 0, value);
		}

		// pseudo-invers
		cvTranspose(Jsum, JsumT);
		cvMatMul(JsumT, Jsum, J);
		cvMatMul(JsumT, ds, JTds);

		cvInvert(J, Jinvers, CV_SVD_SYM);
		cvMatMul(Jinvers, JTds, dx);

		// set
		windage::Matrix3 dxm;
		dxm._33 = 0.0;
		for(int i=0; i<HOMOGRAPHY_COUNT; i++)
			dxm.m1[i] = -2.0 * cvGetReal1D(dx, i);
		homography = homography + dxm;

		// draw result
		windage::Vector3 pt1(0.0, 0.0, 1.0);
		windage::Vector3 pt2(TEMPLATE_WIDTH, 0.0, 1.0);
		windage::Vector3 pt3(TEMPLATE_WIDTH, TEMPLATE_HEIGHT, 1.0);
		windage::Vector3 pt4(0.0, TEMPLATE_HEIGHT, 1.0);

		windage::Vector3 outPoint1 = homography * pt1;
		windage::Vector3 outPoint2 = homography * pt2;
		windage::Vector3 outPoint3 = homography * pt3;
		windage::Vector3 outPoint4 = homography * pt4;

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
		waittingTime = 0;
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