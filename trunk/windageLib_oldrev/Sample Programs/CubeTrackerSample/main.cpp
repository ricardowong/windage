#define RUNNING
#ifdef RUNNING

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

#define FLIP
#define RECTIFICATION
#define ADAPTIVE_THRESHOLD

//#define USE_IMAGE_SEQUENCE

const int FIND_FEATURE_COUNT = 20;

const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 20;
const int ADAPTIVE_THRESHOLD_VALUE = 800;
const int THRESHOLD_STEP = 1;

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::CubeTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::CubeTracker* tracker = new windage::CubeTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, 640, 480, 160, 2.0, 4);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 5, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
//	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	// saving
	bool saving = false;
	CvVideoWriter* writer = NULL;

	char message[100];
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	
	// Tracker Initialize
	IplImage* referenceImage = cvLoadImage("CUBEreference.png", 0);
	windage::CubeTracker* tracker = CreateTracker(referenceImage, 0);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	IplImage* grabFrame = NULL;

	bool processing = true;
	cvNamedWindow("result");
	while(processing)
	{
		// camera frame grabbing and convert to gray and undistortion
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, grabFrame);
		cvResize(grabFrame, tempImage);
		calibration->Undistortion(tempImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();

		tracker->SetFeatureExtractThreshold(fastThreshold);
		tracker->UpdateCameraPose(grayImage);
		int featureCount = tracker->GetFeatureCount();
		int matchingCount = tracker->GetMatchedCount();

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();
			
		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif
		
		// draw tracking result
		if(featureCount > FIND_FEATURE_COUNT)
		{
			tracker->DrawOutLine(inputImage, true);
			tracker->DrawInfomation(inputImage, 100.0);
			tracker->DrawDebugInfo(inputImage);
		}

		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count : %d", matchingCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
		for(int i=0; i<6; i++)
		{
			sprintf(message, "#%d:%d ", i, tracker->GetMatchedCount(i));
			windage::Utils::DrawTextToImage(inputImage, cvPoint(220 + 65*i, 70), message);
		}

		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 's':
		case 'S':
			saving = true;
			if(writer) cvReleaseVideoWriter(&writer);
			writer = cvCreateVideoWriter("saveimage\\capture.avi", CV_FOURCC_DEFAULT, 30, cvSize(inputImage->width, inputImage->height), 1);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
	}

	if(writer) cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}

#endif