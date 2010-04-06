#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <windage.h>

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;

void main()
{
	// capture image
	IplImage* inputImage = NULL;
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SURFdetector();
//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::WSURFMultidetector(WIDTH, HEIGHT);
	detector->SetThreshold(60);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	int index = 0;
	char message[100];
	bool flip = true;
	bool processing = true;
	while(processing)
	{
		// grab image
		inputImage = cvRetrieveFrame(capture);
		if(flip)
			cvFlip(inputImage, inputImage);
		cvResize(inputImage, resultImage);		
		cvCvtColor(resultImage, grayImage, CV_BGR2GRAY);

		int64 startTime = cvGetTickCount();

//		cvSmooth(grayImage, grayImage, CV_GAUSSIAN, 3, 3);
		detector->DoExtractKeypointsDescriptor(grayImage);

		int64 endTime = cvGetTickCount();
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << processingTime << " ms" << std::endl;

		detector->DrawKeypoints(resultImage);

		char message[100];
		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-25), 0.5, message);

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

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
