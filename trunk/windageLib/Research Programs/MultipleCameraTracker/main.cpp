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

#include "PGRCamera.h"

#define ADAPTIVE_THRESHOLD
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double REAL_WIDTH = 26.70;
const double REAL_HEIGHT = 20.00;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Vector3 GetCameraTranslation(windage::Calibration* fromCalibration, windage::Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.x = cvGetReal2D(fromExtrinsicMatrix, 0, 3);
	fromTranslation.y = cvGetReal2D(fromExtrinsicMatrix, 1, 3);
	fromTranslation.z = cvGetReal2D(fromExtrinsicMatrix, 2, 3);

	windage::Vector3 toTranslation;
	toTranslation.x = cvGetReal2D(toExtrinsicMatrix, 0, 3);
	toTranslation.y = cvGetReal2D(toExtrinsicMatrix, 1, 3);
	toTranslation.z = cvGetReal2D(toExtrinsicMatrix, 2, 3);

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}

	windage::Matrix3 toRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = cvGetReal2D(toExtrinsicMatrix, y, x);
		}
	}
/*
	fromRotation = fromRotation.Inverse();
	toRotation = toRotation.Inverse();

	fromTranslation = fromRotation * fromTranslation;
	toTranslation = toRotation * toTranslation;
//*/
	double x = toTranslation.x - fromTranslation.x;
	double y = toTranslation.y - fromTranslation.y;
	double z = toTranslation.z - fromTranslation.z;

	return windage::Vector3(x, y, z);
}

windage::Matrix3 GetCameraRotation(windage::Calibration* fromCalibration, windage::Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}
	
	windage::Matrix3 toRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = cvGetReal2D(toExtrinsicMatrix, y, x);
		}
	}
	
	return fromRotation * toRotation.Transpose();
}

windage::Matrix4 CalculateCameraExtrinsicParameter(windage::Calibration* fromCalibration, windage::Matrix3 toRotation, windage::Vector3 toTranslation)
{
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.v[0] = cvGetReal2D(fromExtrinsicMatrix, 0, 3);
	fromTranslation.v[1] = cvGetReal2D(fromExtrinsicMatrix, 1, 3);
	fromTranslation.v[2] = cvGetReal2D(fromExtrinsicMatrix, 2, 3);

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}

//	fromTranslation = toRotation * fromTranslation;

	windage::Matrix3 rotation = toRotation * fromRotation;
	windage::Vector3 translation = fromTranslation + toTranslation;

	// set rotatino and translation
	windage::Matrix4 matrix;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			matrix.m[y][x] = rotation.m[y][x];
		}
		matrix.m[y][3] = translation.v[y];
	
	}

	matrix.m[3][0] = matrix.m[3][1] = matrix.m[3][2] = 0.0;
	matrix.m[3][3] = 1.0;

	return matrix;
}

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, REAL_WIDTH, REAL_HEIGHT, 8.0, 8);
	tracker->SetPoseEstimationMethod(windage::PROSAC);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CPGRCamera* capture1 = new CPGRCamera();
	capture1->open();
	capture1->start();
	CPGRCamera* capture2 = new CPGRCamera();
	capture2->open();
	capture2->start();

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
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	IplImage* grabFrame1 = NULL;
	IplImage* grabFrame2 = NULL;

	bool updating = false;
	
	bool processing = true;
	cvNamedWindow("result1");
	cvNamedWindow("result2");
	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		capture1->update();
		capture2->update();
		grabFrame1 = capture1->GetIPLImage();
		grabFrame2 = capture2->GetIPLImage();
		cvCvtColor(grabFrame1, undistImage1, CV_BGRA2BGR);
		cvCvtColor(grabFrame2, undistImage2, CV_BGRA2BGR);
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
#ifdef ADAPTIVE_THRESHOLD
		int featureCount1 = tracker1->GetFeatureCount();
		int featureCount2 = tracker2->GetFeatureCount();
		int featureCount = (featureCount1 + featureCount2) / 2.0;
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

		// draw tracking result
		int matchedCount1 = tracker1->GetMatchedCount();
		if(matchedCount1 > FIND_FEATURE_COUNT)
		{
			tracker1->DrawOutLine(inputImage1, true);
			tracker1->DrawInfomation(inputImage1, 10.0);
			tracker1->DrawDebugInfo(inputImage1);
		}
		int matchedCount2 = tracker2->GetMatchedCount();
		if(matchedCount2 > FIND_FEATURE_COUNT)
		{
			tracker2->DrawOutLine(inputImage2, true);
			tracker2->DrawInfomation(inputImage2, 10.0);
			tracker2->DrawDebugInfo(inputImage2);
		}
		
//*
		// update
		if(matchedCount1 > FIND_FEATURE_COUNT && matchedCount2 > FIND_FEATURE_COUNT && updating)
		{
			rotation12 = GetCameraRotation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter());
			translation12 = GetCameraTranslation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter());
			rotation21 = GetCameraRotation(tracker2->GetCameraParameter(), tracker1->GetCameraParameter());
			translation21 = GetCameraTranslation(tracker2->GetCameraParameter(), tracker1->GetCameraParameter());
			updating = false;
		}

		if(matchedCount1 > FIND_FEATURE_COUNT)
		{
			windage::Matrix4 extrinsic = CalculateCameraExtrinsicParameter(tracker1->GetCameraParameter(), rotation12.Transpose(), translation12);
			calibration->SetExtrinsicMatrix(extrinsic.m1);

			windage::Vector3 temp = windage::Vector3(0.0, 0.0, 0.0);
			CvPoint center = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z);
			CvPoint centerX = calibration->ConvertWorld2Image(temp.x + 5.0, temp.y, temp.z);
			CvPoint centerY = calibration->ConvertWorld2Image(temp.x, temp.y + 5.0, temp.z);
			CvPoint centerZ = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z + 5.0);

			// draw outline
			CvScalar color = CV_RGB(255, 255, 0);
			cvLine(inputImage2, calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage2, calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), color, 5);

			cvLine(inputImage2, center, centerX, CV_RGB(255, 0, 0), 5);
			cvLine(inputImage2, center, centerY, CV_RGB(0, 255, 0), 5);
			cvLine(inputImage2, center, centerZ, CV_RGB(0, 0, 255), 5);
		}
		if(matchedCount2 > FIND_FEATURE_COUNT)
		{
			windage::Matrix4 extrinsic = CalculateCameraExtrinsicParameter(tracker2->GetCameraParameter(), rotation21.Transpose(), translation21);
			calibration->SetExtrinsicMatrix(extrinsic.m1);

			windage::Vector3 temp = windage::Vector3(0.0, 0.0, 0.0);
			CvPoint center = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z);
			CvPoint centerX = calibration->ConvertWorld2Image(temp.x + 5.0, temp.y, temp.z);
			CvPoint centerY = calibration->ConvertWorld2Image(temp.x, temp.y + 5.0, temp.z);
			CvPoint centerZ = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z + 5.0);

			// draw outline
			CvScalar color = CV_RGB(255, 255, 0);
			cvLine(inputImage1, calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(+REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), color, 5);
			cvLine(inputImage1, calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, +REAL_HEIGHT/2.0, 0.0), 
								calibration->ConvertWorld2Image(-REAL_WIDTH/2.0, -REAL_HEIGHT/2.0, 0.0), color, 5);

			cvLine(inputImage1, center, centerX, CV_RGB(255, 0, 0), 5);
			cvLine(inputImage1, center, centerY, CV_RGB(0, 255, 0), 5);
			cvLine(inputImage1, center, centerZ, CV_RGB(0, 0, 255), 5);
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

		char ch = cvWaitKey(1);
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

		cvShowImage("result1", inputImage1);
		cvShowImage("result2", inputImage2);
	}

	capture1->stop();
	capture2->stop();
	delete capture1;
	delete capture2;
	cvDestroyAllWindows();
}
