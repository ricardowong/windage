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

#define ADAPTIVE_THRESHOLD

const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;
const int TRAINING_WIDTH = 200;
const int TRAINING_HEIGHT = 200;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

CvPoint mousePoint;
bool isClicked = false;
void OnMouseEvent( int event, int x, int y, int flags, void* param )
{
	switch(event)
	{
	case CV_EVENT_MOUSEMOVE:
		mousePoint.x = x;
		mousePoint.y = y;
		break;
	case CV_EVENT_LBUTTONDOWN:
		mousePoint.x = x;
		mousePoint.y = y;
		isClicked = true;
		break;
	default:
		break;
	}
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
	IplImage* undistImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	// Multipel tracker Initialize
	std::vector<IplImage*> trainingImage;
	IplImage* tempGrayImage = cvCreateImage(cvSize(TRAINING_WIDTH, TRAINING_HEIGHT), IPL_DEPTH_8U, 1);

	windage::MultipleSURFTracker* multipleTracker = new windage::MultipleSURFTracker();
	multipleTracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	multipleTracker->InitializeOpticalFlow(WIDTH, HEIGHT, cvSize(8, 8), 3);
	multipleTracker->SetDetectIntervalTime(1.0);
	multipleTracker->SetPoseEstimationMethod(windage::RANSAC);
	multipleTracker->SetOutlinerRemove(true);
	multipleTracker->SetRefinement(true);
	multipleTracker->SetFeatureExtractThreshold(30);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	IplImage* grabFrame = NULL;
	
	bool processing = true;
	bool addding = false;
	cvNamedWindow("result");
	cvSetMouseCallback("result", OnMouseEvent);
	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, undistImage);
		calibration->Undistortion( undistImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		multipleTracker->SetFeatureExtractThreshold(fastThreshold);
		multipleTracker->UpdateCameraPose(grayImage);

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();

		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		int featureCount = multipleTracker->GetFeatureCount();
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

		// runtime learning trakcer
		if(addding)
		{
			if(isClicked)
			{
				cvSetImageROI(inputImage, cvRect(mousePoint.x - TRAINING_WIDTH/2, mousePoint.y - TRAINING_HEIGHT/2, TRAINING_WIDTH, TRAINING_HEIGHT));
				cvCvtColor(inputImage, tempGrayImage, CV_BGR2GRAY); 
				multipleTracker->SetFeatureExtractThreshold(30);
				multipleTracker->AttatchReferenceImage(tempGrayImage, tempGrayImage->width, tempGrayImage->height, 4.0, 8);
				cvResetImageROI(inputImage);

//				addding = false;
				isClicked = false;
			}
			cvRectangle(inputImage, cvPoint(mousePoint.x - TRAINING_WIDTH/2, mousePoint.y - TRAINING_HEIGHT/2), cvPoint(mousePoint.x + TRAINING_WIDTH/2, mousePoint.y + TRAINING_HEIGHT/2), CV_RGB(255, 0, 0), 3);
		}

		// draw tracking result
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			int matchedCount = multipleTracker->GetMatchedCount(i);
			if(matchedCount > FIND_FEATURE_COUNT)
			{
				multipleTracker->DrawOutLine(inputImage, i, true);
				multipleTracker->DrawInfomation(inputImage, i, TRAINING_WIDTH / 2.0);
//				multipleTracker->DrawDebugInfo(inputImage);

				CvPoint center = multipleTracker->GetCameraParameter(i)->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				center.x += 10;
				center.y += 10;
				sprintf(message, "Reference #%d", i);
				windage::Utils::DrawTextToImage(inputImage, center, message);
			}

		}
		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count ");
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			sprintf(message, "#%d:%d ", i, multipleTracker->GetMatchedCount(i));
			windage::Utils::DrawTextToImage(inputImage, cvPoint(160 + 65*i, 70), message);
		}

		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'a':
		case 'A':
			addding = !addding;
			isClicked = false;
			break;
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
