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

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int FEATURE_COUNT = WIDTH*2;

const double SCALE_FACTOR = 4.0;
const int SCALE_STEP = 10;
const double REPROJECTION_ERROR = 5.0;

const char* TEMPLATE_IMAGE = "D:\\Datasets\\metaio\\target6\\target6_320.png";
const char* IMAGE_SEQUENCE = "D:\\Datasets\\metaio\\target6\\6-2\\%04d.png";

const double INTRINSIC[] = {901.155, 901.155, 322.1, 229.541, 0, 0, 0, 0};

void main()
{
	windage::Logger logger(&std::cout);

	IplImage* grabImage;
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	cvNamedWindow("result");

	// create and initialize tracker
	windage::Frameworks::PlanarObjectTracking tracking;
	windage::Calibration* calibration;
	windage::Algorithms::FeatureDetector* detector;
	windage::Algorithms::SearchTree* searchtree;
	windage::Algorithms::OpticalFlow* opticalflow;
	windage::Algorithms::HomographyEstimator* estimator;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::HomographyRefiner* refiner;

	calibration = new windage::Calibration();
	detector = new windage::Algorithms::WSURFdetector();
	searchtree = new windage::Algorithms::KDtree();
	opticalflow = new windage::Algorithms::OpticalFlow();
	estimator = new windage::Algorithms::RANSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR * 3);
	refiner->SetMaxIteration(10);

	tracking.AttatchCalibration(calibration);
	tracking.AttatchDetetor(detector);
	tracking.AttatchMatcher(searchtree);
	tracking.AttatchTracker(opticalflow);
	tracking.AttatchEstimator(estimator);
	tracking.AttatchChecker(checker);
	tracking.AttatchRefiner(refiner);
//	tracking.AttatchFilter(filter);

	tracking.SetDitectionRatio(0);
	tracking.Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	int keypointCount = 0;
	int matchingCount = 0;
	double threshold = 20.0;
	double processingTime = 0.0;

	bool trained = false;

	IplImage* sampleImage = cvLoadImage(TEMPLATE_IMAGE, 0);
	cvSmooth(sampleImage, sampleImage, CV_GAUSSIAN, 5, 5);
	detector->SetThreshold(threshold);
	tracking.AttatchReferenceImage(sampleImage);
	tracking.TrainingReference(SCALE_FACTOR, SCALE_STEP);
	detector->SetThreshold(threshold);
	trained = true;

	char filename[500];
	char message[100];
	bool flip = false;
	bool processing = true;
	for(int l=0; l<1200&&processing; l++)
	{
		// capture image
		sprintf(filename, IMAGE_SEQUENCE, l);
		inputImage = cvLoadImage(filename);
		cvCopyImage(inputImage, resultImage);
		cvCvtColor(resultImage, grayImage, CV_BGR2GRAY);

		logger.updateTickCount();

		// track object
		if(trained)
		{
			tracking.UpdateCamerapose(grayImage);

			// adaptive threshold
			int localcount = detector->GetKeypointsCount();
			if(keypointCount != localcount)
			{
				if(localcount > FEATURE_COUNT)
					threshold += 1;
				if(localcount < FEATURE_COUNT)
					threshold -= 1;
				detector->SetThreshold(threshold);
				keypointCount = localcount;
			}

			// draw result
//			detector->DrawKeypoints(resultImage);
			tracking.DrawOutLine(resultImage, true);
			tracking.DrawDebugInfo(resultImage);
			calibration->DrawInfomation(resultImage, 100);
		}
		matchingCount = tracking.GetMatchingCount();

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
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

		cvReleaseImage(&inputImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}		
	}

//	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
