#include <iostream>

#include "PGRCamera.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "Reconstructor/Reconstructor.h"
#include "Utils/Utils.h"

const int WIDTH = 640;
const int HEIGHT = 480;

CvScalar worldPoint = cvScalar(0, 0, 0);
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
	// connect camera
	CPGRCamera* camera1 = new CPGRCamera();
	CPGRCamera* camera2 = new CPGRCamera();
	camera1->open();
	camera2->open();
	camera1->start();
	camera2->start();

	IplImage* input1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	cvNamedWindow("image1");
	cvSetMouseCallback("image1",MouseEvent1);
	cvNamedWindow("image2");
	cvSetMouseCallback("image2",MouseEvent2);

	// initialize tracker
	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	windage::Tracker* tracker1 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker1)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker1)->SetOpticalFlowRunning(true);
	
	windage::Tracker* tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 5, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);

	bool processing = true;
	while(processing)
	{
		// camera frame grabbing
		camera1->update();
		camera2->update();

		cvResize(camera1->GetIPLImage(), input1);
		cvResize(camera2->GetIPLImage(), input2);
		cvCvtColor(input1, gray1, CV_BGRA2GRAY);
		cvCvtColor(input2, gray2, CV_BGRA2GRAY);

		// call tracking algorithm
		tracker1->UpdateCameraPose(gray1);
		tracker2->UpdateCameraPose(gray2);

		tracker1->DrawInfomation(input1, 10.0);
		tracker2->DrawInfomation(input2, 10.0);

		// draw mouse points
		cvDrawCircle(input1, lPoint, 10, CV_RGB(255, 0, 0), 2);
		cvDrawCircle(input2, rPoint, 10, CV_RGB(255, 0, 0), 2);

		// calcuate reconstruction
		worldPoint = windage::Reconstructor::Calc3DPointApproximation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter(), lPoint, rPoint);
		windage::Utils::DrawWorldCoordinatePoint(input1, tracker1->GetCameraParameter(), worldPoint, 1.0, true);
		windage::Utils::DrawWorldCoordinatePoint(input2, tracker2->GetCameraParameter(), worldPoint, 1.0, true);

		cvShowImage("image1", input1);
		cvShowImage("image2", input2);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	camera1->stop();
	camera1->close();
	camera2->stop();
	camera2->close();
}