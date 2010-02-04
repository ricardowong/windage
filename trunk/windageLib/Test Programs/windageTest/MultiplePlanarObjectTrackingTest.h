/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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
#include "Frameworks/MultiplePlanarObjectTracking.h"

class MultiplePlanarObjectTrackingTest : public windageTest
{
private:
	IplImage* inputImage1;
	IplImage* inputImage2;
	IplImage* grayImage1;
	IplImage* grayImage2;
	CvSize imageSize;

	windage::Calibration* calibration;

	windage::Algorithms::WSURFdetector* detector;
	windage::Algorithms::RANSACestimator* estimator;
	windage::Algorithms::OpticalFlow* tracker;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::LMmethod* refiner;

public:
	MultiplePlanarObjectTrackingTest() : windageTest("MultiplePlanarObjectTracking Test", "MultiplePlanarObjectTracking")
	{
		inputImage1 = NULL;
		inputImage2 = NULL;
		grayImage1 = NULL;
		grayImage2 = NULL;

		detector = NULL;
		estimator = NULL;
		refiner = NULL;

		this->Do();
	}
	~MultiplePlanarObjectTrackingTest()
	{
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;

		if(calibration) delete calibration;
		calibration = NULL;
		if(detector) delete detector;
		detector = NULL;
		if(tracker) delete tracker;
		tracker = NULL;
		if(estimator) delete estimator;
		estimator = NULL;
		if(checker) delete checker;
		checker = NULL;
		if(refiner) delete refiner;
		refiner = NULL;
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

		calibration = new windage::Calibration();
		detector = new windage::Algorithms::WSURFdetector();
		tracker = new windage::Algorithms::OpticalFlow();
		estimator = new windage::Algorithms::RANSACestimator();
		checker = new windage::Algorithms::OutlierChecker();
		refiner = new windage::Algorithms::LMmethod();

		calibration->Initialize(1200, 1200, 200, 160, 0, 0, 0, 0);
		tracker->Initialize(imageSize.width, imageSize.height);
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

		windage::Frameworks::MultiplePlanarObjectTracking* tracking1 = new windage::Frameworks::MultiplePlanarObjectTracking();
		p1 = (void*)tracking1;
		tracking1->AttatchCalibration(this->calibration);
		tracking1->AttatchDetetor(this->detector);
		tracking1->AttatchTracker(this->tracker);
		tracking1->AttatchEstimator(this->estimator);
		tracking1->AttatchChecker(this->checker);
		tracking1->AttatchRefiner(this->refiner);
		tracking1->Initialize(imageSize.width, imageSize.height);
		tracking1->AttatchReferenceImage(grayImage1);
		tracking1->AttatchReferenceImage(grayImage1);
		tracking1->AttatchReferenceImage(grayImage1);
		tracking1->TrainingReference();
		tracking1->UpdateCamerapose(grayImage2);
		tracking1->UpdateCamerapose(grayImage2);
		tracking1->UpdateCamerapose(grayImage2);
		tracking1->UpdateCamerapose(grayImage2);
		tracking1->UpdateCamerapose(grayImage2);
		tracking1->UpdateCamerapose(grayImage2);
		delete tracking1;

		windage::Frameworks::MultiplePlanarObjectTracking* tracking2 = new windage::Frameworks::MultiplePlanarObjectTracking();
		p2 = (void*)tracking2;
		tracking2->AttatchCalibration(this->calibration);
		tracking2->AttatchDetetor(this->detector);
		tracking2->AttatchTracker(this->tracker);
		tracking2->AttatchEstimator(this->estimator);
		tracking2->AttatchChecker(this->checker);
		tracking2->AttatchRefiner(this->refiner);
		tracking2->Initialize(imageSize.width, imageSize.height);
		tracking2->AttatchReferenceImage(grayImage1);
		tracking2->TrainingReference();
		tracking2->UpdateCamerapose(grayImage2);
		delete tracking2;		

		sprintf_s(memoryAddress1, "%08X", p1);
		sprintf_s(memoryAddress2, "%08X", p2);
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
		cvNamedWindow("Multiple Object Tracking Frameworks");

		windage::Frameworks::MultiplePlanarObjectTracking tracking;
		tracking.AttatchCalibration(this->calibration);
		tracking.AttatchDetetor(this->detector);
		tracking.AttatchTracker(this->tracker);
		tracking.AttatchEstimator(this->estimator);
		tracking.AttatchChecker(this->checker);
		tracking.AttatchRefiner(this->refiner);
		tracking.Initialize(width, height, width, height);
		tracking.AttatchReferenceImage(grayImage1);
		tracking.AttatchReferenceImage(grayImage2);
		tracking.TrainingReference();

		tracking.UpdateCamerapose(grayImage2);
		tracking.UpdateCamerapose(grayImage2);
		tracking.UpdateCamerapose(grayImage2);

		for(int i=0; i<tracking.GetObjectCount(); i++)
		{
			tracking.DrawDebugInfo(resultImage, i);
			tracking.DrawOutLine(resultImage, i, true);
			tracking.GetCameraParameter(i)->DrawInfomation(resultImage, 100.0);
		}
		
		cvShowImage("Multiple Object Tracking Frameworks", resultImage);
		cvWaitKey(3000);

		sprintf_s(tempMessage, "");
		(*message) = std::string(tempMessage);
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
		
		cvDestroyWindow("Multiple Object Tracking Frameworks");

		return true;
	}
};

