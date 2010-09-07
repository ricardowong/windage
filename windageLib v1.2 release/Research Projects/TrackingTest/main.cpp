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
#include "../Common/FleaCamera.h"

#define WRITE_VIDEO 1

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int FEATURE_COUNT = WIDTH;

const double SCALE_FACTOR = 4.0;
const int SCALE_STEP = 8;
const double REPROJECTION_ERROR = 10.0;

#define USE_ADAPTIVE_THRESHOLD 0
#define USE_TEMPLATE_IMAEG 1
const char* TEMPLATE_IMAGE = "reference1_320.png";
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

windage::Frameworks::PlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::PlanarObjectTracking* tracking = new windage::Frameworks::PlanarObjectTracking();

	windage::Calibration* calibration;
	windage::Algorithms::FeatureDetector* detector;
	windage::Algorithms::SearchTree* searchtree;
	windage::Algorithms::OpticalFlow* opticalflow;
	windage::Algorithms::HomographyEstimator* estimator;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::HomographyRefiner* refiner;
	windage::Algorithms::KalmanFilter* filter;

	calibration = new windage::Calibration();
	detector = new windage::Algorithms::WSURFdetector();
	searchtree = new windage::Algorithms::KDtree();
	opticalflow = new windage::Algorithms::OpticalFlow();
	estimator = new windage::Algorithms::RANSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();
	filter = new windage::Algorithms::KalmanFilter();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR * 3);
	refiner->SetMaxIteration(10);

	tracking->AttatchCalibration(calibration);
	tracking->AttatchDetetor(detector);
	tracking->AttatchMatcher(searchtree);
	tracking->AttatchTracker(opticalflow);
	tracking->AttatchEstimator(estimator);
	tracking->AttatchChecker(checker);
// 	tracking->AttatchRefiner(refiner);
//	tracking->AttatchFilter(filter);

	tracking->SetDitectionRatio(1);
	tracking->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	return tracking;
}

void main()
{
	windage::Logger logger1("RANSAC", "txt");
	windage::Logger logger2("ProSAC", "txt");

	IplImage* grabImage;
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

#if WRITE_VIDEO
	CvVideoWriter* writer = cvCreateVideoWriter("tracking_test.avi", CV_FOURCC_DEFAULT, 15, cvSize(WIDTH, HEIGHT));
#endif

	FleaCamera* capture = new FleaCamera();
	capture->open();
	capture->start();
//	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	// create and initialize tracker
	windage::Frameworks::PlanarObjectTracking* tracker1 = CreateTracker();
	windage::Frameworks::PlanarObjectTracking* tracker2 = CreateTracker();
	tracker2->AttatchEstimator(new windage::Algorithms::ProSACestimator());
	tracker2->GetEstimator()->SetReprojectionError(REPROJECTION_ERROR);
	
	int keypointCount = 0;
	int matchingCount = 0;
	double threshold = tracker1->GetDetector()->GetThreshold();
	double processingTime1 = 0.0;
	double processingTime2 = 0.0;

	bool trained = false;

#if USE_TEMPLATE_IMAEG
	IplImage* sampleImage = cvLoadImage(TEMPLATE_IMAGE, 0);
	tracker1->AttatchReferenceImage(sampleImage);
	tracker1->TrainingReference(SCALE_FACTOR, SCALE_STEP);
	tracker2->AttatchReferenceImage(sampleImage);
	tracker2->TrainingReference(SCALE_FACTOR, SCALE_STEP);
	trained = true;
#endif

	char message[100];
	bool flip = false;
	bool processing = true;
	while(processing)
	{
		// capture image
		capture->update();
		grabImage = capture->GetIPLImage();
//		inputImage = cvRetrieveFrame(capture);
		cvResize(grabImage, inputImage);
		cvCvtColor(inputImage, resultImage, CV_BGRA2BGR);
		cvCvtColor(resultImage, grayImage, CV_BGR2GRAY);
		if(flip)
			cvFlip(inputImage, inputImage);

		logger1.updateTickCount();
		tracker1->UpdateCamerapose(grayImage);
		tracker1->DrawOutLine(resultImage, true);
		processingTime1 = logger1.calculateProcessTime();

		logger2.updateTickCount();
		tracker2->UpdateCamerapose(grayImage);
		tracker2->DrawOutLine(resultImage, CV_RGB(0, 255, 255), true);
		processingTime2 = logger2.calculateProcessTime();

		matchingCount = tracker1->GetMatchingCount();
		
		logger1.log("processingTime RANSAC", processingTime1);
		logger1.logNewLine();
		logger2.log("processingTime ProSAC", processingTime2);
		logger2.logNewLine();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime1);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", keypointCount, threshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

		sprintf_s(message, "Press 'Space' to track the current image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-25), 0.5, message);
		cvShowImage("result", resultImage);

#if WRITE_VIDEO
		cvWriteToAVI(writer, resultImage);
#endif

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		case 'f':
		case 'F':
			flip = !flip;
			break;
		}		
	}

	capture->stop();
	capture->close();
	delete capture;

#if WRITE_VIDEO
	cvReleaseVideoWriter(&writer);
#endif

//	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
