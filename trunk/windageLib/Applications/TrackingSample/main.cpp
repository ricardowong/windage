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

const int WIDTH = 320;
const double RATIO =(double)WIDTH / 640.0;
const int HEIGHT = 480*RATIO;

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	// saving
	bool saving = false;
	CvVideoWriter* writer = NULL;

	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT*2), IPL_DEPTH_8U, 3);

	IplImage* referenceImage = NULL;
	IplImage* referenceColor = NULL;
	IplImage* trainingImage = NULL;

	// Tracker Initialize
	if(WIDTH == 640)
	{
		referenceImage = cvLoadImage("reference_map.png", 0);
		referenceColor = cvLoadImage("reference_map.png");
		trainingImage = cvLoadImage("reference_map_320.png", 0);
	}
	else if(WIDTH == 320)
	{
		referenceImage = cvLoadImage("reference_map_320.png", 0);
		referenceColor = cvLoadImage("reference_map_320.png");
		trainingImage = cvLoadImage("reference_map_320.png", 0);
	}
	else if(WIDTH == 1024)
	{
		referenceImage = cvLoadImage("reference_map_1024.png", 0);
		referenceColor = cvLoadImage("reference_map_1024.png");
		trainingImage = cvLoadImage("reference_map_320.png", 0);
	}

	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	if(WIDTH == 1024)
	{
		tracker->Initialize(1330.260, 1328.958, 549.620, 374.955, -0.188632, 0.090592, -0.000546, -0.001740, 50);
	}
	else
	{
		tracker->Initialize(778.195*RATIO, 779.430*RATIO, 324.659*RATIO, 235.685*RATIO, -0.333103, 0.173760, 0.000653, 0.001114, 50);
	}
	tracker->RegistReferenceImage(trainingImage, 26.70, 20.00, 4.0, 8);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	tracker->SetOpticalFlowRunning(false);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractTreshold(30);

	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 130;
	const int MIN_FAST_THRESHOLD = 30;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	IplImage* grabFrame = NULL;
	
	char message[100];
	fpslog->updateTickCount();
	bool processing = true;
#ifdef USE_IMAGE_SEQUENCE
	cvNamedWindow("result");
	for(int i=0; i<4735; i++)
	{
		log->updateTickCount();
		char* filenameTemplate = "D:\\ImageSequence\\20090912-2\\capture%d.jpg";
		char filename[300];
		sprintf(filename, filenameTemplate, i);

		if(grabFrame) cvReleaseImage(&grabFrame);
		grabFrame = cvLoadImage(filename);
#else
	cvNamedWindow("result");
	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
#ifdef FLIP
		cvFlip(grabFrame, grabFrame);
#endif
#endif

		cvResize(grabFrame, tempImage);
#ifdef RECTIFICATION
		tracker->GetCameraParameter()->Undistortion(tempImage, inputImage);
#else
		cvCopy(grabFrame, inputImage);
#endif

		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);

//		cvCopy(referenceColor, inputImage);
//		cvCopy(referenceImage, grayImage);
		
//		cvSmooth(grayImage, grayImage, 2, 1, 1);

		log->log("capture", log->calculateProcessTime());

		fpslog->updateTickCount();
		// call tracking algorithm
		log->updateTickCount();
		int result = tracker->UpdateCameraPose(grayImage);

		int featureCount = tracker->GetFeatureCount();
		int matchedCount = tracker->GetMatchedCount();

		// for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE ) fastThreshold+=THRESHOLD_STEP;
		else fastThreshold-=THRESHOLD_STEP;
		if(fastThreshold > MAX_FAST_THRESHOLD) fastThreshold = MAX_FAST_THRESHOLD;
		if(fastThreshold < MIN_FAST_THRESHOLD) fastThreshold = MIN_FAST_THRESHOLD;
#endif
		tracker->SetFeatureExtractTreshold(fastThreshold);

		double fps = fpslog->calculateFPS();
		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->log("matchedCount", matchedCount);
		log->logNewLine();
		// draw tracking result
		//tracker->DrawDebugInfo(inputImage);
		tracker->DrawOutLine(inputImage, true);
		tracker->DrawInfomation(inputImage, 5.0);
//*
		cvSetImageROI(tempImage2, cvRect(0, 0, WIDTH, HEIGHT));
		cvCopy(referenceColor, tempImage2);
		cvSetImageROI(tempImage2, cvRect(0, HEIGHT, WIDTH, HEIGHT));
		cvCopy(inputImage, tempImage2);
		cvResetImageROI(tempImage2);
		tracker->DrawDebugInfo2(tempImage2);

		CvRect rect = cvRect(10, HEIGHT-10-HEIGHT/2, WIDTH/4, HEIGHT/2);
		cvRectangle(inputImage, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width, rect.y+rect.height), CV_RGB(255, 255, 255), 5);
		cvSetImageROI(inputImage, rect);
		cvResize(tempImage2, inputImage);
		cvResetImageROI(inputImage);

//		cvNamedWindow("temp");
//		cvShowImage("temp", tempImage2);
//*/
		
		sprintf(message, "FPS : %03.2f, Time : %.2f(ms)", fps, trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), message);

		sprintf(message, "Feature : %03d, Matched : %03d", featureCount, matchedCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 40), message);

		if(saving)
		{
			if(writer)
				cvWriteFrame(writer, inputImage);
			else
				writer = cvCreateVideoWriter("saveimage\\capture.avi", CV_FOURCC_DEFAULT, 30, cvSize(WIDTH, HEIGHT), 1);
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		case ' ':
			cvWaitKey();
			break;
		case 's':
			saving = true;
			break;
		}

		cvShowImage("result", inputImage);
	}

	if(writer) cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
}

#endif