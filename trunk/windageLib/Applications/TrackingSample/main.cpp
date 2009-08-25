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

#include "PGRCamera.h"
#include "Tracker/ChessboardTracker.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "Utils/Logger.h"
#include "Utils/Utils.h"

#define NATURAL_FEATURE_TRACKING
#define RECTIFICATION

//#define IMAGE_320_240
#define IMAGE_640_480

#ifdef IMAGE_320_240
const int WIDTH = 320;
const int HEIGHT = 240;
#else
const int WIDTH = 640;
const int HEIGHT = 480;
#endif


void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	// Tracker Initialize
#ifdef NATURAL_FEATURE_TRACKING
	IplImage* referenceImage = cvLoadImage("reference.png", 0);

	windage::Tracker* tracker = new windage::ModifiedSURFTracker();
#ifdef IMAGE_320_240
	((windage::ModifiedSURFTracker*)tracker)->Initialize(389.0975, 389.715, 162.3295, 117.8425, -0.333103, 0.173760, 0.000653, 0.001114, 20);
#else
	((windage::ModifiedSURFTracker*)tracker)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
#endif
	((windage::ModifiedSURFTracker*)tracker)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker)->SetOpticalFlowRunning(true);
#else
	windage::Tracker* tracker = new windage::ChessboardTracker();
#ifdef IMAGE_320_240
	((windage::ChessboardTracker*)tracker)->Initialize(389.0975, 389.715, 162.3295, 117.8425, -0.333103, 0.173760, 0.000653, 0.001114, 7, 8, 2.80);
#else
	((windage::ChessboardTracker*)tracker)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 7, 8, 2.80);
#endif
#endif

	// connect camera
	CPGRCamera* camera = new CPGRCamera();
	camera->open();
	camera->start();

	cvNamedWindow("result");
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);

	char message[100];

	fpslog->updateTickCount();
	bool processing = true;
	while(processing)
	{
		double fps = fpslog->calculateFPS();
		fpslog->updateTickCount();
		
		// camera frame grabbing
		log->updateTickCount();
		camera->update();

#ifdef RECTIFICATION
		tracker->GetCameraParameter()->Undistortion(camera->GetIPLImage(), tempImage);
#else
		cvCopy(camera->GetIPLImage(), tempImage);
#endif
		cvResize(tempImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		int result = tracker->UpdateCameraPose(grayImage);

		// draw tracking result
		tracker->DrawDebugInfo(inputImage);
		tracker->DrawInfomation(inputImage, 10.0);

		log->log("tracking", log->calculateProcessTime());

		log->log("result", result);
		log->logNewLine();

		sprintf(message, "FPS : %lf, Feature Count : %d", fps, result);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), message);

		cvShowImage("result", inputImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	camera->stop();
	camera->close();
}