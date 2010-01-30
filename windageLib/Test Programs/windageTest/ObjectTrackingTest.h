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

#include <cv.h>
#include <highgui.h>

#include "windageTest.h"

#include "windage.h"
#include "Frameworks/ObjectTracking.h"

class ObjectTrackingTest : public windageTest
{
private:
	IplImage* inputImage1;
	IplImage* inputImage2;
	IplImage* grayImage1;
	IplImage* grayImage2;
	CvSize imageSize;

public:
	ObjectTrackingTest() : windageTest("ObjectTracking Test", "ObjectTracking")
	{
		inputImage1 = NULL;
		inputImage2 = NULL;
		grayImage1 = NULL;
		grayImage2 = NULL;

		this->Do();
	}
	~ObjectTrackingTest()
	{
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters

		// load reference image
		inputImage1 = cvLoadImage(REFERENCE_IMAGE_FILENAME.c_str());
		inputImage2 = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		imageSize = cvGetSize(inputImage1);

		grayImage1 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		grayImage2 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		resultImage = cvCreateImage(imageSize, IPL_DEPTH_8U, 3);

		cvCvtColor(inputImage1, grayImage1, CV_BGR2GRAY);
		cvCvtColor(inputImage2, grayImage2, CV_BGR2GRAY);

		return true;
	}

	bool TestMemoryRelease(std::string* message)
	{
		// checek the memory leak
		const int size = 10;
		char memoryAddress1[size];
		char memoryAddress2[size];

		void* p1 = 0;
		void* p2 = 0;
		int compair = 0;


		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		(*message) = std::string(memoryAddress1) + std::string(",") + std::string(memoryAddress2);
		if(compair == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool TestAlgorithm(std::string* message)
	{
		bool test = true;
		char tempMessage[100];

		int width = grayImage1->width;
		int height = grayImage1->height;

		cvCopyImage(inputImage2, resultImage);
		cvNamedWindow("Object Tracking Frameworks");

		windage::Algorithms::WSURFdetector detector;
		windage::Algorithms::FLANNtree matcher;
		windage::Algorithms::RANSACestimator estimator;

		windage::Frameworks::ObjectTracking tracking;
		tracking.AttatchDetetor(&detector);
		tracking.AttatchMatcher(&matcher);
		tracking.AttatchEstimator(&estimator);

		tracking.Initialize(width, height);
		tracking.AttatchReferenceImage(grayImage1);
		tracking.UpdateCamerapose(grayImage2);

		windage::Vector3 drawRefPoints[4];
		windage::Vector3 drawScePoints[4];
		drawRefPoints[0].x = 0.0;	drawRefPoints[0].y = 0.0;		drawRefPoints[0].z = 1.0;
		drawRefPoints[1].x = width; drawRefPoints[1].y = 0.0;		drawRefPoints[1].z = 1.0;
		drawRefPoints[2].x = width; drawRefPoints[2].y = height;	drawRefPoints[2].z = 1.0;
		drawRefPoints[3].x = 0.0;	drawRefPoints[3].y = height;	drawRefPoints[3].z = 1.0;

		for(int i=0; i<4; i++)
		{
			drawScePoints[i] = estimator.ConvertObjectToImage(drawRefPoints[i]);
			drawScePoints[i] /= drawScePoints[i].z;
		}

		for(int i=0; i<4; i++)
		{
			int i2 = i==3?0:i+1;
			cvLine(resultImage, cvPoint(drawScePoints[i].x, drawScePoints[i].y),
								cvPoint(drawScePoints[i2].x, drawScePoints[i2].y), CV_RGB(0, 255, 0), 3);
		}		
		
		cvShowImage("Object Tracking Frameworks", resultImage);
		cvWaitKey(3000);

		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;
		
		cvDestroyWindow("Object Tracking Frameworks");

		return true;
	}
};

