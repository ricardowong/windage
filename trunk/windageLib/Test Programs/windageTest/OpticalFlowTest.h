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
#include "Algorithms/WSURFdetector.h"
#include "Algorithms/OpticalFlow.h"
#include "Utilities/Utils.h"

class OpticalFlowTest : public windageTest
{
private:
	IplImage* inputImage1;
	IplImage* inputImage2;
	IplImage* grayImage1;
	IplImage* grayImage2;
	CvSize imageSize;

	windage::Algorithms::WSURFdetector* detector;

public:
	OpticalFlowTest() : windageTest("OpticalFlow Test", "OpticalFlow")
	{
		inputImage1 = NULL;
		inputImage2 = NULL;
		grayImage1 = NULL;
		grayImage2 = NULL;

		detector = NULL;
		this->Do();
	}
	~OpticalFlowTest()
	{
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;

		if(detector) delete detector;
		detector = NULL;
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters
		inputImage1 = cvLoadImage(REFERENCE_IMAGE_FILENAME.c_str());
		inputImage2 = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		imageSize = cvGetSize(inputImage1);

		grayImage1 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		grayImage2 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		resultImage = cvCreateImage(imageSize, IPL_DEPTH_8U, 3);

		cvCvtColor(inputImage1, grayImage1, CV_BGR2GRAY);
		cvCvtColor(inputImage2, grayImage2, CV_BGR2GRAY);

		detector = new windage::Algorithms::WSURFdetector();
		detector->DoExtractKeypointsDescriptor(grayImage1);

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

		std::vector<windage::FeaturePoint> currPoints;
		std::vector<windage::FeaturePoint>* prevPoints = detector->GetKeypoints();

		// Feature Point
		windage::Algorithms::OpticalFlow* tracker1 = new windage::Algorithms::OpticalFlow();
		p1 = (void*)tracker1;
		tracker1->Initialize(imageSize.width, imageSize.height);
		tracker1->Initialize(imageSize.width, imageSize.height);
		tracker1->TrackFeatures(grayImage1, grayImage2, prevPoints, &currPoints);
		tracker1->TrackFeatures(grayImage1, grayImage2, prevPoints, &currPoints);
		delete tracker1;

		currPoints.clear();

		windage::Algorithms::OpticalFlow* tracker2 = new windage::Algorithms::OpticalFlow();
		p2 = (void*)tracker2;
		tracker2->Initialize(imageSize.width, imageSize.height);
		tracker2->TrackFeatures(grayImage1, grayImage2, prevPoints, &currPoints);
		delete tracker2;

		currPoints.clear();

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

		cvNamedWindow("OpticalFlow");

		windage::Algorithms::OpticalFlow tracker;
		tracker.Initialize(imageSize.width, imageSize.height);

		IplImage* color[2];
		color[0] = inputImage1;
		color[1] = inputImage2;
		
		IplImage* image[2];
		image[0] = grayImage1;
		image[1] = grayImage2;

		std::vector<windage::FeaturePoint>* points[2];
		std::vector<windage::FeaturePoint> currPoints;

		points[0] = detector->GetKeypoints();
		points[1] = &currPoints;

		int index1, index2;
		for(int i=0; i<10; i++)
		{
			index1 = i%2;
			index2 = (index1 + 1)%2;

			cvCopyImage(color[index2], resultImage);

			tracker.TrackFeatures(image[index1], image[index2], points[index1], points[index2]);

			for(int i=0; i<points[index1]->size(); i++)
			{
				windage::Vector3 tempPt1 = (*points[index2])[i].GetPoint();
				windage::Vector3 tempPt2 = (*points[index1])[i].GetPoint();
				cvCircle(resultImage, cvPoint(tempPt1.x, tempPt1.y), 3, CV_RGB(255, 0, 0));
				cvLine(resultImage, cvPoint(tempPt1.x, tempPt1.y), cvPoint(tempPt2.x, tempPt2.y), CV_RGB(255, 0, 0));
			}
			points[index1]->clear();

			cvShowImage("OpticalFlow", resultImage);
			cvWaitKey(100);
		}
		
		
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

		if(detector) delete detector;
		detector = NULL;

		cvDestroyWindow("OpticalFlow");

		return true;
	}
};

