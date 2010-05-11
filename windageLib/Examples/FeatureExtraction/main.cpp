#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/FleaCamera.h"

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;

void main()
{
	// capture image
	IplImage* grabImage = NULL;
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SURFdetector();
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::WSURFdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::WSURFMultidetector(WIDTH, HEIGHT);
	detector->SetThreshold(60);

	FleaCamera* capture = new FleaCamera();
	capture->open();
	capture->start();
//	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	double threshold = 30.0;
	int index = 0;
	char message[100];
	bool flip = false;
	bool processing = true;
	while(processing)
	{
		// grab image
		capture->update();
		grabImage = capture->GetIPLImage();
		cvResize(grabImage, inputImage);
		cvCvtColor(inputImage, resultImage, CV_BGRA2BGR);
//		inputImage = cvRetrieveFrame(capture);
		if(flip)
			cvFlip(inputImage, inputImage);
//		cvResize(inputImage, resultImage);
		cvCvtColor(resultImage, grayImage, CV_BGR2GRAY);

		int64 startTime = cvGetTickCount();

//		cvSmooth(grayImage, grayImage, CV_GAUSSIAN, 3, 3);
		detector->SetThreshold(threshold);
		detector->DoExtractKeypointsDescriptor(grayImage);

		int64 endTime = cvGetTickCount();
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << processingTime << " ms" << std::endl;

		detector->DrawKeypoints(resultImage);

		char message[100];
		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Threshold : %.2lf ms", threshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);

		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-200, HEIGHT-25), 0.5, message);

		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'f':
		case 'F':
			flip = !flip;
			break;
		case 'q':
		case 'Q':
		case 27:
			processing = false;
			break;
		case '+':
			threshold *= 2.0;
			break;
		case '-':
			threshold /= 2.0;
			break;
		case 's':
		case 'S':
			{
				sprintf_s(message, "save/image%03d_original.png", index);
				cvSaveImage(message, inputImage);

				sprintf_s(message, "save/image%03d_featureinfo.png", index);
				cvSaveImage(message, resultImage);

				sprintf_s(message, "save/descriptor%03d", index);
				windage::Logger* featureLogger = new windage::Logger(message, "txt");

				index++;

				windage::FeatureExportor exportor;
				exportor.AttatchLogger(featureLogger);
				exportor.SetFunctionName(detector->GetFunctionName());
				exportor.SetFeaturePoints(detector->GetKeypoints());

				exportor.DoExport();
				delete featureLogger;
				featureLogger = NULL;
			}
			break;
		}
	}

	capture->stop();
	capture->close();
	delete capture;
//	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
