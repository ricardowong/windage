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

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int FEATURE_COUNT = WIDTH*2;

void main()
{
	windage::Logger logger(&std::cout);

	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
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
	searchtree = new windage::Algorithms::FLANNtree();
	opticalflow = new windage::Algorithms::OpticalFlow();
	estimator = new windage::Algorithms::ProSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();

	calibration->Initialize(WIDTH*1.2, WIDTH*1.2, WIDTH/2.0, HEIGHT/2.0, 0, 0, 0, 0);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(8, 8), 3);
	estimator->SetReprojectionError(10.0);
	checker->SetReprojectionError(10.0);
	refiner->SetMaxIteration(10);

	tracking.AttatchCalibration(calibration);
	tracking.AttatchDetetor(detector);
	tracking.AttatchMatcher(searchtree);
	tracking.AttatchTracker(opticalflow);
	tracking.AttatchEstimator(estimator);
	tracking.AttatchChecker(checker);
	tracking.AttatchRefiner(refiner);

	tracking.SetDitectionRatio(30);
	tracking.Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);
	
	int keypointCount = 0;
	int matchingCount = 0;
	double threshold = 50.0;
	double processingTime = 0.0;

	char message[100];
	bool trained = false;
	bool processing = true;
	while(processing)
	{
		// capture image
		inputImage = cvRetrieveFrame(capture);
		cvResize(inputImage, resizeImage);
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

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
			tracking.DrawDebugInfo(resultImage);
			tracking.DrawOutLine(resultImage, true);
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

		sprintf_s(message, "Press 'Space' to track the current image", keypointCount, threshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		case ' ':
		case 's':
		case 'S':
			detector->SetThreshold(30.0);
			tracking.AttatchReferenceImage(grayImage);
			tracking.TrainingReference(4.0, 8);
			detector->SetThreshold(threshold);
			trained = true;
			break;
		}		
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
