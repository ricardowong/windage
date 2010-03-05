#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <windage.h>

const int WIDTH = 640;
const int HEIGHT = 480;

void main()
{
	// capture image
	IplImage* inputImage = NULL;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	bool processing = true;
	while(processing)
	{
		// grab image
		inputImage = cvRetrieveFrame(capture);
		cvFlip(inputImage, inputImage);
		cvResize(inputImage, resizeImage);		
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);

		int64 startTime = cvGetTickCount();

		detector->DoExtractKeypointsDescriptor(grayImage);

		int64 endTime = cvGetTickCount();
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << processingTime << " ms" << std::endl;

		detector->DrawKeypoints(resizeImage);

		cvShowImage("result", resizeImage);
		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
		case 27:
			processing = false;
			break;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
