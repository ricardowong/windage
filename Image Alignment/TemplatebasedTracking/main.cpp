/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo
 *   U-VR Lab, GIST of Gwangju in Korea.
 *   http://windage.googlecode.com/
 *   http://uvr.gist.ac.kr/
 *
 * Copyright of the derived and new portions of this work
 *     (C) 2009 GIST U-VR Lab.
 *
 * This framework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information please contact 
 *   Woonhyuk Baek
 *   <windage@live.com>
 *   GIST U-VR Lab.
 *   Department of Information and Communication
 *   Gwangju Institute of Science and Technology
 *   1, Oryong-dong, Buk-gu, Gwangju
 *   South Korea
 * ========================================================================
 ** @author   Woonhyuk Baek
 * ======================================================================== */

#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "../Utils/Utils.h"
#include "../Algorithms/homographyESM.h"
#include "../Algorithms/InverseCompositional.h"

#define USE_ESM 0
#define USE_IC 1

const int GAUSSIAN_BLUR = 3;

const int WIDTH = 640;
const int HEIGHT = 480;

const int TEMPLATE_WIDTH = 300;
const int TEMPLATE_HEIGHT = 300;
const double HOMOGRAPHY_DELTA = 0.01;
const int MAX_ITERATION = 50;

void DrawResult(IplImage* image, windage::Matrix3 homography, CvScalar color = CV_RGB(255, 0, 0), int thickness = 1)
{
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

	cvLine(image, cvPoint(outPoint1.x, outPoint1.y), cvPoint(outPoint2.x, outPoint2.y), color, thickness);
	cvLine(image, cvPoint(outPoint2.x, outPoint2.y), cvPoint(outPoint3.x, outPoint3.y), color, thickness);
	cvLine(image, cvPoint(outPoint3.x, outPoint3.y), cvPoint(outPoint4.x, outPoint4.y), color, thickness);
	cvLine(image, cvPoint(outPoint4.x, outPoint4.y), cvPoint(outPoint1.x, outPoint1.y), color, thickness);
}

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("sampling");
	cvNamedWindow("result");

	// initialize
	int width = WIDTH;
	int height = HEIGHT;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	
	IplImage* inputImage = NULL;
	IplImage* grayImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage* templateImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* samplingImage = NULL;

	// initial template & homography
	CvRect rect = cvRect(startX, startY, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	windage::Matrix3 homography(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	homography._13 = startX;
	homography._23 = startY;
	windage::Matrix3 e = homography;

	// Template based Tracking using Inverse Compositional
#if USE_IC
	windage::InverseCompositional* tracker = new windage::InverseCompositional(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
#endif
#if USE_ESM
	windage::HomographyESM* tracker = new windage::HomographyESM(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
#endif
	tracker->SetInitialHomography(e);

	// homography update stack
	std::vector<windage::Matrix3> homographyList;

	// camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	bool isTrained = false;
	bool processing =true;
	while(processing)
	{
		inputImage = cvRetrieveFrame(capture);
		cvResize(inputImage, resultImage);
		cvCvtColor(resultImage, grayImage, CV_BGR2GRAY);
		
		if(GAUSSIAN_BLUR > 0)
			cvSmooth(grayImage, grayImage, CV_GAUSSIAN, GAUSSIAN_BLUR, GAUSSIAN_BLUR);

		// processing
		int64 startTime = cvGetTickCount();
		
		double error = 0.0;
		double delta = 1.0;
		int iter = 0;
		homographyList.clear();
		for(iter=0; iter<MAX_ITERATION; iter++)
		{
			error = tracker->UpdateHomography(grayImage, &delta);
			homography = tracker->GetHomography();
			homographyList.push_back(homography);

			if(delta < HOMOGRAPHY_DELTA)
				break;
		}
		int64 endTime = cvGetTickCount();
		samplingImage = tracker->GetSamplingImage();

		// draw result
		int count = homographyList.size();
		for(int i=0; i<count; i++)
 			DrawResult(resultImage, homographyList[i], CV_RGB(((count-i)/(double)count) * 255.0, (i/(double)count) * 255.0, 0), 1);
 		
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		sprintf(message, "processing time : %.2lf ms (%02d iter), error : %.2lf", processingTime, iter, error);
		std::cout << message << std::endl;

#if USE_IC
		windage::Utils::DrawTextToImage(resultImage, cvPoint(5, 15), "Inverse Compositional", 0.6);
#endif
#if USE_ESM
		windage::Utils::DrawTextToImage(resultImage, cvPoint(5, 15), "Efficient Second-order Minimization", 0.6);
#endif
		windage::Utils::DrawTextToImage(resultImage, cvPoint(5, 35), message, 0.6);

		// draw image
		cvShowImage("sampling", samplingImage);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			cvSetImageROI(grayImage, rect);
			cvCopyImage(grayImage, templateImage);
			cvShowImage("template", templateImage);
			cvResetImageROI(grayImage);

			tracker->AttatchTemplateImage(templateImage);
			tracker->SetInitialHomography(e);
			tracker->Initialize();
			break;
		case 'r':
		case 'R':
			delete tracker;
			tracker = NULL;
#if USE_IC
			tracker = new windage::InverseCompositional(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
#endif
#if USE_ESM
			tracker = new windage::HomographyESM(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
#endif
			tracker->SetInitialHomography(e);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

	}

	cvReleaseCapture(&capture);

	cvReleaseImage(&grayImage);
	cvReleaseImage(&resultImage);
	cvDestroyAllWindows();
}
