#include <iostream>

#include "PGRCamera.h"
#include "Tracker/ChessboardTracker.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "Utils/Logger.h"

const int WIDTH = 640;
const int HEIGHT = 480;

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// 640 x 480
//	windage::Tracker* tracker = new windage::ChessboardTracker();
//	((windage::ChessboardTracker*)tracker)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 2.80);

	// 320 x 240
	//tracker->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 28.0);

	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	windage::Tracker* tracker = new windage::ModifiedSURFTracker();
//	((windage::ModifiedSURFTracker*)tracker)->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, referenceImage, 281.0, 211.0, 75);
	((windage::ModifiedSURFTracker*)tracker)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, referenceImage, 281.0, 211.0, 75);
	((windage::ModifiedSURFTracker*)tracker)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	((windage::ModifiedSURFTracker*)tracker)->SetOpticalFlowRunning(true);


	CPGRCamera* camera = new CPGRCamera();
	camera->open();
	camera->start();

	cvNamedWindow("test");
//	IplImage* input = cvCloneImage(camera->GetIPLImage());
	IplImage* input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray = cvCreateImage(cvGetSize(input), IPL_DEPTH_8U, 1);

	bool processing = true;
	while(processing)
	{
		log->updateTickCount();
		camera->update();
		cvResize(camera->GetIPLImage(), input);
		cvCvtColor(input, gray, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		log->updateTickCount();
		int result = tracker->UpdateCameraPose(gray);
		tracker->DrawDebugInfo(input);
		tracker->DrawInfomation(input, 100.0);
		log->log("tracking", log->calculateProcessTime());

		log->log("result", result);
		log->logNewLine();
		
		cvShowImage("test", input);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	camera->stop();
	camera->close();
}