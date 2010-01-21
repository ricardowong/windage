#include <iostream>

#include <cv.h>
#include <highgui.h>

const int IMAGE_SEQ_COUNT = 200;
const char* IMAGE_SEQ_FILE_NAME = "seq/im%03d.pgm";

const double PROCESSING_TIME = 33.0;//ms

void  main()
{
	char message[100];
	cvNamedWindow("input");

	// initialize

	
	bool processing =true;
	for(int i=0; i<IMAGE_SEQ_COUNT && processing; i++)
	{
		int64 startTime = cvGetTickCount();

		// load image
		sprintf(message, IMAGE_SEQ_FILE_NAME, i);
		IplImage* inputImage = cvLoadImage(message, 0);

		// processing

		// draw image
		cvShowImage("input", inputImage);
		cvReleaseImage(&inputImage);

		int64 endTime = cvGetTickCount();
		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << i << " : processing time : " << processingTime << " ms" << std::endl;

		int waittingTime = cvRound(PROCESSING_TIME - processingTime);
		if(waittingTime < 1) waittingTime = 1;
		char ch = cvWaitKey(waittingTime);

		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

}