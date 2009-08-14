#include <iostream>

#include "PGRCamera.h"
#include "Tracker/ChessboardTracker.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "Reconstructor/Reconstructor.h"
#include "Utils/Logger.h"
#include "Utils/Utils.h"

//#define STEREO_MODE

const int WIDTH = 640;
const int HEIGHT = 480;

CvScalar worldPoint;
CvPoint lPoint, rPoint;

void MouseEvent1( int mevent, int x, int y, int flags, void* param )
{
   if (mevent == CV_EVENT_LBUTTONDOWN)
   {
	   lPoint = cvPoint(x, y);
   }
}
void MouseEvent2( int mevent, int x, int y, int flags, void* param )
{
   if (mevent == CV_EVENT_LBUTTONDOWN)
   {
	   rPoint = cvPoint(x, y);
   }
}


void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	// 640 x 480
/*
	windage::Tracker* tracker = new windage::ChessboardTracker();
	((windage::ChessboardTracker*)tracker)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 2.80);
//*/
	// 320 x 240
//	tracker->Initialize( 535.703,  539.716, 158.839, 098.400, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 28.0);

	IplImage* referenceImage = cvLoadImage("reference.png", 0);
//*
	windage::Tracker* tracker1 = new windage::ModifiedSURFTracker();
//	((windage::ModifiedSURFTracker*)tracker1)->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 30);
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 45);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker1)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker1)->SetOpticalFlowRunning(true);
//*/
	CPGRCamera* camera1 = new CPGRCamera();
	camera1->open();
	camera1->start();

	cvNamedWindow("test1");
	cvSetMouseCallback("test1",MouseEvent1);
	IplImage* input1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray1 = cvCreateImage(cvGetSize(input1), IPL_DEPTH_8U, 1);

	char message[100];


#ifdef STEREO_MODE
	windage::Tracker* tracker2 = new windage::ModifiedSURFTracker();
//	((windage::ModifiedSURFTracker*)tracker2)->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 75);
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 5, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);

	CPGRCamera* camera2 = new CPGRCamera();
	camera2->open();
	camera2->start();

	cvNamedWindow("test2");
	cvSetMouseCallback("test2",MouseEvent2);
	IplImage* input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray2 = cvCreateImage(cvGetSize(input2), IPL_DEPTH_8U, 1);

#endif

	fpslog->updateTickCount();
	bool processing = true;
	while(processing)
	{
		double fps = fpslog->calculateFPS();
		fpslog->updateTickCount();
		
		log->updateTickCount();
		camera1->update();
		cvResize(camera1->GetIPLImage(), input1);
		cvCvtColor(input1, gray1, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		log->updateTickCount();
		int result = tracker1->UpdateCameraPose(gray1);
		tracker1->DrawDebugInfo(input1);
		tracker1->DrawInfomation(input1, 10.0);

		log->log("tracking", log->calculateProcessTime());

		log->log("result", result);
		log->logNewLine();

		sprintf(message, "FPS : %lf, Feature Count : %d", fps, result);
		windage::Utils::DrawTextToImage(input1, cvPoint(10, 20), message);

		windage::Utils::DrawWorldCoordinatePoint(input1, tracker1->GetCameraParameter(), worldPoint, 1.0, true);

		cvDrawCircle(input1, lPoint, 10, CV_RGB(255, 0, 0), 2);
		cvShowImage("test1", input1);

#ifdef STEREO_MODE
		camera2->update();
		cvResize(camera2->GetIPLImage(), input2);
		cvCvtColor(input2, gray2, CV_BGRA2GRAY);

		result = tracker2->UpdateCameraPose(gray2);
//		tracker2->DrawDebugInfo(input2);
		tracker2->DrawInfomation(input2, 10.0);

		windage::Utils::DrawWorldCoordinatePoint(input2, tracker2->GetCameraParameter(), worldPoint, 1.0, true);

		cvDrawCircle(input2, rPoint, 10, CV_RGB(255, 0, 0), 2);
		cvShowImage("test2", input2);
#endif

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'p':
		case 'P':
#ifdef STEREO_MODE
			worldPoint = windage::Reconstructor::Calc3DPointApproximation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter(), lPoint, rPoint);
#endif
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	camera1->stop();
	camera1->close();

#ifdef STEREO_MODE
	camera2->stop();
	camera2->close();
#endif

}