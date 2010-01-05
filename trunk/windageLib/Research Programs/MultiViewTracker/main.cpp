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

#include <highgui.h>
#include <windage.h>

//#define ADAPTIVE_THRESHOLD
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double REAL_WIDTH = 267.0;
const double REAL_HEIGHT = 200.0;

//const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};
// temporary intrinsic values
const double intrinsicValues[8] = {529.400, 528.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, REAL_WIDTH, REAL_HEIGHT, 4.0, 8);
	tracker->SetPoseEstimationMethod(windage::PROSAC);
	tracker->SetOutlinerRemove(true);
	tracker->SetRefinement(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(false);
//	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	return tracker;
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	char message[100];
	IplImage* inputImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* undistImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* inputImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* undistImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// Multipel tracker Initialize
	IplImage* trainingImage = cvLoadImage("reference1_320.png", 0);
	IplImage* referenceImage = cvLoadImage("reference1.png");
	
	windage::ModifiedSURFTracker* tracker1 = CreateTracker(trainingImage, 0);
	windage::ModifiedSURFTracker* tracker2 = CreateTracker(trainingImage, 5);

	// for undistortion and calculate coordinate
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	windage::Matrix3 rotation12;
	windage::Matrix3 rotation21;
	windage::Vector3 translation12;
	windage::Vector3 translation21;	

	// adaptive threshold
	int fastThreshold = 30;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	bool updating = true;
	
	bool processing = true;
	cvNamedWindow("result1");
	cvNamedWindow("result2");
	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();

		// load image -> undistImage
		undistImage1 = cvLoadImage("picture 8.jpg");
		undistImage2 = cvLoadImage("picture 9.jpg");

		calibration->Undistortion(undistImage1, inputImage1);
		calibration->Undistortion(undistImage2, inputImage2);
		cvCvtColor(inputImage1, grayImage1, CV_BGRA2GRAY);
		cvCvtColor(inputImage2, grayImage2, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		tracker1->SetFeatureExtractThreshold(fastThreshold);
		tracker2->SetFeatureExtractThreshold(fastThreshold);
		tracker1->UpdateCameraPose(grayImage1);
		tracker2->UpdateCameraPose(grayImage2);

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();

		// update fast threshold for Adaptive threshold
		int featureCount1 = tracker1->GetFeatureCount();
		int featureCount2 = tracker2->GetFeatureCount();
#ifdef ADAPTIVE_THRESHOLD
		int featureCount = (featureCount1 + featureCount2) / 2.0;
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

		// draw tracking result
		int matchedCount1 = tracker1->GetMatchedCount();
		if(matchedCount1 > FIND_FEATURE_COUNT)
		{
			tracker1->DrawOutLine(inputImage1, true);
			tracker1->DrawInfomation(inputImage1, 100.0);
			tracker1->DrawDebugInfo(inputImage1);
		}
		int matchedCount2 = tracker2->GetMatchedCount();
		if(matchedCount2 > FIND_FEATURE_COUNT)
		{
			tracker2->DrawOutLine(inputImage2, true);
			tracker2->DrawInfomation(inputImage2, 100.0);
			tracker2->DrawDebugInfo(inputImage2);
		}
		
//*
		// update
		if(matchedCount1 > FIND_FEATURE_COUNT && matchedCount2 > FIND_FEATURE_COUNT && updating)
		{
			
			rotation12 = windage::MultiCameraCoordinate::GetRotation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter());
			translation12 = windage::MultiCameraCoordinate::GetTranslation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter());
			rotation21 = windage::MultiCameraCoordinate::GetRotation(tracker2->GetCameraParameter(), tracker1->GetCameraParameter());
			translation21 = windage::MultiCameraCoordinate::GetTranslation(tracker2->GetCameraParameter(), tracker1->GetCameraParameter());
			updating = false;
		}

		if(matchedCount1 > FIND_FEATURE_COUNT)
		{
			windage::Matrix4 extrinsic = windage::MultiCameraCoordinate::CalculateExtrinsic(tracker1->GetCameraParameter(), rotation12.Transpose(), translation12);
			calibration->SetExtrinsicMatrix(extrinsic.m1);

			windage::Vector3 temp = windage::Vector3(0.0, 0.0, 0.0);
			CvPoint center = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z);
			CvPoint centerX = calibration->ConvertWorld2Image(temp.x + 50.0, temp.y, temp.z);
			CvPoint centerY = calibration->ConvertWorld2Image(temp.x, temp.y + 50.0, temp.z);
			CvPoint centerZ = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z + 50.0);

			// draw outline
			CvScalar color = CV_RGB(255, 255, 0);
			cvLine(inputImage2, calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), color, 5);

			cvLine(inputImage2, center, centerX, CV_RGB(255, 0, 0), 10);
			cvLine(inputImage2, center, centerY, CV_RGB(0, 255, 0), 10);
			cvLine(inputImage2, center, centerZ, CV_RGB(0, 0, 255), 10);
		}
		if(matchedCount2 > FIND_FEATURE_COUNT)
		{
			windage::Matrix4 extrinsic = windage::MultiCameraCoordinate::CalculateExtrinsic(tracker2->GetCameraParameter(), rotation21.Transpose(), translation21);
			calibration->SetExtrinsicMatrix(extrinsic.m1);

			windage::Vector3 temp = windage::Vector3(0.0, 0.0, 0.0);
			CvPoint center = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z);
			CvPoint centerX = calibration->ConvertWorld2Image(temp.x + 50.0, temp.y, temp.z);
			CvPoint centerY = calibration->ConvertWorld2Image(temp.x, temp.y + 50.0, temp.z);
			CvPoint centerZ = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z + 50.0);

			// draw outline
			CvScalar color = CV_RGB(255, 255, 0);
			cvLine(inputImage1, calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(+REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, +REAL_HEIGHT/4.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/4.0, -REAL_HEIGHT/4.0, 0.0), color, 5);

			cvLine(inputImage1, center, centerX, CV_RGB(255, 0, 0), 10);
			cvLine(inputImage1, center, centerY, CV_RGB(0, 255, 0), 10);
			cvLine(inputImage1, center, centerZ, CV_RGB(0, 0, 255), 10);
		}
//*/
		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage1, cvPoint(20, 30), message);
		windage::Utils::DrawTextToImage(inputImage2, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount1, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage1, cvPoint(20, 50), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount2, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage2, cvPoint(20, 50), message);
		sprintf(message, "Match count : %d", matchedCount1);
		windage::Utils::DrawTextToImage(inputImage1, cvPoint(20, 70), message);
		sprintf(message, "Match count : %d", matchedCount2);
		windage::Utils::DrawTextToImage(inputImage2, cvPoint(20, 70), message);

		cvShowImage("result1", inputImage1);
		cvShowImage("result2", inputImage2);

		char ch = cvWaitKey();
		switch(ch)
		{
		case 'u':
		case 'U':
			updating = true;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

	}

	cvDestroyAllWindows();
}
