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

#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/BumbleBeeCamera.h"

const int TRACKER_COUNT = 2;
const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int FEATURE_COUNT = WIDTH;

const double SCALE_FACTOR = 4.0;
const int SCALE_STEP = 8;

#define USE_ADAPTIVE_THRESHOLD 1
#define USE_TEMPLATE_IMAEG 1
const char* TEMPLATE_IMAGE = "reference.png";
const double INTRINSIC[] = {826.653, 826.135, 351.964, 262.518, -0.014979, 0.051856, -0.000729, -0.000744};

windage::Frameworks::PlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::PlanarObjectTracking* tracking = new windage::Frameworks::PlanarObjectTracking();

	windage::Calibration* calibration					= new windage::Calibration();
	windage::Algorithms::FeatureDetector* detector		= new windage::Algorithms::WSURFdetector();
	windage::Algorithms::SearchTree* searchtree			= new windage::Algorithms::FLANNtree();
	windage::Algorithms::OpticalFlow* opticalflow		= new windage::Algorithms::OpticalFlow();
	windage::Algorithms::HomographyEstimator* estimator	= new windage::Algorithms::RANSACestimator();
	windage::Algorithms::OutlierChecker* checker		= new windage::Algorithms::OutlierChecker();
	windage::Algorithms::HomographyRefiner* refiner		= new windage::Algorithms::LMmethod();
	windage::Algorithms::KalmanFilter* filter			= new windage::Algorithms::KalmanFilter();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(8, 8), 3);
	estimator->SetReprojectionError(5.0);
	checker->SetReprojectionError(5.0);
	refiner->SetMaxIteration(5);

	tracking->AttatchCalibration(calibration);
	tracking->AttatchDetetor(detector);
	tracking->AttatchMatcher(searchtree);
	tracking->AttatchTracker(opticalflow);
	tracking->AttatchEstimator(estimator);
	tracking->AttatchChecker(checker);
	tracking->AttatchRefiner(refiner);
//	tracking->AttatchFilter(filter);

	CvRNG rng = cvRNG(cvGetTickCount());

	int ratio = 5;
	tracking->SetDitectionRatio(ratio);
	tracking->SetFilterSetp(cvRandInt(&rng)%ratio);
	tracking->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);
	return tracking;
}

void main()
{
	windage::Logger logger(&std::cout);

	std::vector<windage::Vector3> toTranslation; toTranslation.resize(TRACKER_COUNT);
	std::vector<windage::Matrix3> toRotation; toRotation.resize(TRACKER_COUNT);
	std::vector<IplImage*> inputImage;
	std::vector<IplImage*> grayImage;
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH*2, HEIGHT), IPL_DEPTH_8U, 4);
	for(int i=0; i<TRACKER_COUNT; i++)
	{
		inputImage.push_back(cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4));
		grayImage.push_back(cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1));
	}

	BumbleBeeCamera* stereoCamera = new BumbleBeeCamera();
	unsigned char* dummy1 = NULL;
	unsigned char* dummy2 = NULL;
	stereoCamera->init(WIDTH, HEIGHT, dummy1, dummy2);
	stereoCamera->SetColorBuffers((unsigned char*)inputImage[0]->imageData, (unsigned char*)inputImage[1]->imageData);
	cvNamedWindow("result");

	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	// create and initialize tracker
	std::vector<windage::Frameworks::PlanarObjectTracking*> tracker;
	for(int i=0; i<TRACKER_COUNT; i++)
	{
		tracker.push_back(CreateTracker());
	}

	double processingTime = 0.0;
#if USE_TEMPLATE_IMAEG
	IplImage* sampleImage = cvLoadImage(TEMPLATE_IMAGE, 0);

	for(unsigned int i=0; i<tracker.size(); i++)
	{
		tracker[i]->GetDetector()->SetThreshold(30.0);
		tracker[i]->AttatchReferenceImage(sampleImage);
		tracker[i]->TrainingReference(SCALE_FACTOR, SCALE_STEP);
		tracker[i]->GetDetector()->SetThreshold(50.0);
	}
#endif

	int cameraIndex = -1;
	char message[100];
	bool update = false;
	bool processing = true;
	while(processing)
	{
		// capture image
		stereoCamera->capture();
		stereoCamera->download();

		logger.updateTickCount();

		for(int i=0; i<TRACKER_COUNT; i++)
		{
			if(cameraIndex != i)
			{
				cvCvtColor(inputImage[i], grayImage[i], CV_BGRA2GRAY);
				tracker[i]->UpdateCamerapose(grayImage[i]);
				tracker[i]->DrawOutLine(inputImage[i], true);

//				tracker[i]->GetCameraParameter()->DrawInfomation(inputImage[i], 100.0);
			}
		}
		for(int i=0; i<TRACKER_COUNT; i++)
		{
			int index = i+1>=TRACKER_COUNT?0:i+1;
			calibration->SetExtrinsicMatrix(windage::Coordinator::MultiCameraCoordinator::CalculateExtrinsic(tracker[i]->GetCameraParameter(), toRotation[i], toTranslation[i]).m1);
			calibration->DrawInfomation(inputImage[index], 100.0);
		}

		// draw result
		cvSetImageROI(resultImage, cvRect(0, 0, WIDTH, HEIGHT));
		cvCopyImage(inputImage[0], resultImage);
		cvSetImageROI(resultImage, cvRect(WIDTH, 0, WIDTH, HEIGHT));
		cvCopyImage(inputImage[1], resultImage);
		cvResetImageROI(resultImage);

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		
		sprintf_s(message, "Bounding : tracking object");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH*2-270, HEIGHT-10), 0.7, message);
		sprintf_s(message, "Axis : estimated pose");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH*2-270, HEIGHT-30), 0.7, message);

		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 27:
		case 'q':
		case 'Q':
			processing = false;
			break;
		case 'u':
		case 'U':
			for(int i=0; i<TRACKER_COUNT; i++)
			{
				int index = i+1>=TRACKER_COUNT?0:i+1;
				toTranslation[i] = windage::Coordinator::MultiCameraCoordinator::GetTranslation(tracker[i]->GetCameraParameter(), tracker[index]->GetCameraParameter());
				toRotation[i] = windage::Coordinator::MultiCameraCoordinator::GetRotation(tracker[i]->GetCameraParameter(), tracker[index]->GetCameraParameter());
			}
			break;
		case '1':
			cameraIndex = 0;
			break;
		case '2':
			cameraIndex = 1;
			break;
		case '0':
		case '3':
			cameraIndex = -1;
			break;
/*
		case ' ':
		case 's':
		case 'S':

			tracker[0]->GetDetector()->SetThreshold(30.0);
			tracker[0]->AttatchReferenceImage(grayImage);
			tracker[0]->TrainingReference(SCALE_FACTOR, SCALE_STEP);
			tracker[0]->GetDetector()->SetThreshold(threshold);
*/
			break;
		}		
	}

	stereoCamera->clean();
	delete stereoCamera;
	cvDestroyAllWindows();
}
