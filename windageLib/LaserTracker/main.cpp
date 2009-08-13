
#include "PGRCamera.h"
#include "Tracker/ChessboardTracker.h"

void main()
{
	windage::ChessboardTracker* tracker = new windage::ChessboardTracker();

	// 640 x 480
	tracker->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 2.80);

	// 320 x 240
	//tracker->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 28.0);

	CPGRCamera* camera = new CPGRCamera();
	camera->open();
	camera->start();

	cvNamedWindow("test");
	IplImage* input = cvCloneImage(camera->GetIPLImage());

	bool processing = true;
	while(processing)
	{
		camera->update();
		cvCopyImage(camera->GetIPLImage(), input);

		tracker->UpdateCameraPose(input);
		tracker->DrawInfomation(input);
		
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
}