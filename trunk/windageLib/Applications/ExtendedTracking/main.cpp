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

#include <windows.h >

#include <omp.h>
#include <iostream>
#include <vector>

#include <highgui.h>
#include <windage.h>

#include "KDTreeWrapper.h"
#include "PlaneEstimation.h"

#define RECTIFICATION

const int WIDTH = 640;
const int HEIGHT = 480;
const double ORIGINAL_WIDTH = 267.0;
const double ORIGINAL_HEIGHT = 200.0;

IplImage* src;
IplImage* dst;
windage::Calibration* calibration;

void AddPanoramaImage(IplImage* src, IplImage* dst, windage::Calibration* calibration)
{
	int width = dst->width;
	int height = dst->height;

	int boundLeft = 0;
	int boundRight = width;
	int boundTop = 0;
	int boundDown = height;
	
	// find roi range
	int tx=0;
	int ty=0;
	CvPoint2D64f tempPoint[4];
	
	tempPoint[0] = calibration->ConvertImage2World(0.0, 0.0, 0.0);
	tempPoint[0].x += (double)width/2.0;
	tempPoint[0].y += (double)height/2.0;
	tempPoint[0].x = MAX(0.0, tempPoint[0].x);
	tempPoint[0].x = MIN((double)width, tempPoint[0].x);
	tempPoint[0].y = MAX(0.0, tempPoint[0].y);
	tempPoint[0].y = MIN((double)height, tempPoint[0].y);

	tempPoint[1] = calibration->ConvertImage2World((double)src->width, 0.0, 0.0);
	tempPoint[1].x += (double)width/2.0;
	tempPoint[1].y += (double)height/2.0;
	tempPoint[1].x = MAX(0.0, tempPoint[1].x);
	tempPoint[1].x = MIN((double)width, tempPoint[1].x);
	tempPoint[1].y = MAX(0.0, tempPoint[1].y);
	tempPoint[1].y = MIN((double)height, tempPoint[1].y);

	tempPoint[2] = calibration->ConvertImage2World(0.0, (double)src->height, 0.0);
	tempPoint[2].x += (double)width/2.0;
	tempPoint[2].y += (double)height/2.0;
	tempPoint[2].x = MAX(0.0, tempPoint[2].x);
	tempPoint[2].x = MIN((double)width, tempPoint[2].x);
	tempPoint[2].y = MAX(0.0, tempPoint[2].y);
	tempPoint[2].y = MIN((double)height, tempPoint[2].y);

	tempPoint[3] = calibration->ConvertImage2World((double)src->width, (double)src->height, 0.0);
	tempPoint[3].x += (double)width/2.0;
	tempPoint[3].y += (double)height/2.0;
	tempPoint[3].x = MAX(0.0, tempPoint[3].x);
	tempPoint[3].x = MIN((double)width, tempPoint[3].x);
	tempPoint[3].y = MAX(0.0, tempPoint[3].y);
	tempPoint[3].y = MIN((double)height, tempPoint[3].y);

	boundLeft	= MIN(MIN((int)tempPoint[0].x, (int)tempPoint[1].x), MIN((int)tempPoint[2].x, (int)tempPoint[3].x));
	boundRight	= MAX(MAX((int)tempPoint[0].x, (int)tempPoint[1].x), MAX((int)tempPoint[2].x, (int)tempPoint[3].x));
	boundTop	= MIN(MIN((int)tempPoint[0].y, (int)tempPoint[1].y), MIN((int)tempPoint[2].y, (int)tempPoint[3].y));
	boundDown	= MAX(MAX((int)tempPoint[0].y, (int)tempPoint[1].y), MAX((int)tempPoint[2].y, (int)tempPoint[3].y));

//	#pragma omp parallel for schedule(dynamic)
	for(int y=boundTop; y<boundDown; y++)
	{
		for(int x=boundLeft; x<boundRight; x++)
		{
			CvPoint point = calibration->ConvertWorld2Image(x-width/2, y-height/2, 0.0);
			if( 0 < point.x && point.x < src->width &&
				0 < point.y && point.y < src->height)
			{
				CvScalar color = cvGet2D(src, point.y, point.x);
				cvSet2D(dst, height-y-1, x, color);
			}
		}
	}
}


DWORD WINAPI WorkerThread(LPVOID)
{
	AddPanoramaImage(src, dst, calibration);
	return 0;
} 

void main()
{
	Logger* log = new Logger(&std::cout);
	Logger* logFPS = new Logger(&std::cout);

	cvNamedWindow("reference");
	cvNamedWindow("result");

	// camera capture
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	CvVideoWriter* writer = NULL;
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);

	// default Tracker Initialize
	IplImage* grabFrame = NULL;
	IplImage* referenceImage = cvLoadImage("reference1.png", 0);
	IplImage* referenceColor = cvLoadImage("reference1_320.png");

	windage::ModifiedSURFTracker* defaultTracker = new windage::ModifiedSURFTracker();
	defaultTracker = new windage::ModifiedSURFTracker();
	defaultTracker->Initialize(1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769, 50);
//	defaultTracker->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 50);
	defaultTracker->RegistReferenceImage(referenceImage, ORIGINAL_WIDTH, ORIGINAL_HEIGHT, 4.0, 8);
	defaultTracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	defaultTracker->SetPoseEstimationMethod(windage::PROSAC);
	defaultTracker->SetOutlinerRemove(true);
	defaultTracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8 ), 3);
	defaultTracker->SetOpticalFlowRunning(true);

	// panorama reference image
	IplImage* panoramaImage = cvCreateImage(cvSize(WIDTH*2, HEIGHT*2), IPL_DEPTH_8U, 1);
	IplImage* panoramaColor = cvCreateImage(cvSize(WIDTH*2, HEIGHT*2), IPL_DEPTH_8U, 3);
	double ratio = WIDTH / (double)panoramaColor->width;
	IplImage* panoramaResize = cvCreateImage(cvSize(panoramaColor->width * ratio, panoramaColor->height * ratio), IPL_DEPTH_8U, 3);

	// for thread
	src = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	dst = panoramaColor;
	calibration = new windage::Calibration();
	calibration->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114);

	// for adaptive threshold
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 130;
	const int MIN_FAST_THRESHOLD = 30;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	int index = 0;

	char message[100];
	bool saving = false;
	bool processing = true;
	while(processing)
	{
		logFPS->updateTickCount();
		// capture the image
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, grabFrame);
		defaultTracker->GetCameraParameter()->Undistortion(grabFrame, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);
		log->log("capture", log->calculateProcessTime());

		// update camera pose
		log->updateTickCount();
		defaultTracker->UpdateCameraPose(grayImage);
		int featureCount = defaultTracker->GetFeatureCount();
		int matchingCount = defaultTracker->GetMatchedCount();
		log->log("tracking", log->calculateProcessTime());

		// adaptive threshold
		if(featureCount < ADAPTIVE_THRESHOLD_VALUE) fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
		else										fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		defaultTracker->SetFeatureExtractTreshold(fastThreshold);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q' :
		case 'Q' :
			processing = false;
			break;
		case 'a':
		case 'A':
			AddPanoramaImage(inputImage, panoramaColor, defaultTracker->GetCameraParameter());
			break;
		case 'e':
		case 'E':
			cvCvtColor(panoramaColor, panoramaImage, CV_BGR2GRAY);
			defaultTracker->SetFeatureExtractTreshold(30);
			defaultTracker->RegistReferenceImage(panoramaImage, panoramaImage->width, panoramaImage->height, 2.0, 4);
			break;
		case 'c':
		case 'C':
			defaultTracker->RegistReferenceImage(grayImage, grayImage->width, grayImage->height, 2.0, 4);
			break;
		case 's':
		case 'S':
			cvSaveImage("panoramaImage.png", panoramaColor);
			saving = true;
			break;
		default:
			if(matchingCount > 50)
			{
				index++; if(index > 30) index = 0;
				if(index == 15)
				{
					cvCopyImage(inputImage, src);
//					cvCopyImage(panoramaColor, dst);
					calibration->SetExtrinsicMatrix(defaultTracker->GetCameraParameter()->GetExtrinsicMatrix());

					DWORD dwID;
					CreateThread(NULL, 0, WorkerThread, NULL, 0, &dwID);
//					AddPanoramaImage(inputImage, panoramaColor, defaultTracker->GetCameraParameter());
				}
			}
			break;
		}

		if(saving)
		{
			if(writer)
				cvWriteFrame(writer, panoramaResize);
			else
				writer = cvCreateVideoWriter("saveimage\\capture.avi", CV_FOURCC_DEFAULT, 30, cvSize(panoramaResize->width, panoramaResize->height), 1);
		}		

		// draw result
		defaultTracker->DrawOutLine(inputImage, true);
		defaultTracker->DrawInfomation(inputImage, 50.0);
		defaultTracker->DrawDebugInfo(inputImage);
		sprintf(message, "FPS : %.2lf", logFPS->calculateFPS());
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), message);
		sprintf(message, "feature : %d (%d), matching : %d", featureCount, fastThreshold, matchingCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 40), message);
		cvShowImage("result", inputImage);

		cvResize(panoramaColor, panoramaResize);
		cvShowImage("reference", panoramaResize);

		log->logNewLine();

	}

	if(writer) cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}