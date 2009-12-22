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
#include <highgui.h>
#include <windage.h>

const int OBJECT_COUNT = 6;

const int WIDTH1 = 320;
const int HEIGHT1 = 240;
const int BORDER = 10;
const int WIDTH = (WIDTH1 + BORDER) * OBJECT_COUNT + BORDER;
const int HEIGHT = HEIGHT1 + BORDER*2;

std::vector<IplImage*> canvasImage;
IplImage* inputImage;
CvScalar ALPHA_MASK_COLOR = CV_RGB(0, 255, 255);

bool isUpdating = false;

void TrackbarCallback(int pos)
{
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	char message[500];
	inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	
	// Multiple tracker Initialize
	std::vector<IplImage*> trainingImage;
	for(int i=0; i<OBJECT_COUNT; i++)
	{
		sprintf(message, "reference%d_320.png", i+1);
		trainingImage.push_back(cvLoadImage(message));

		canvasImage.push_back(cvCreateImage(cvSize(WIDTH1, HEIGHT1), IPL_DEPTH_8U, 3));
		cvSet(canvasImage[i], ALPHA_MASK_COLOR);

		cvSetImageROI(inputImage, cvRect((WIDTH1 + BORDER) * (i) + BORDER, BORDER, WIDTH1, HEIGHT1));
		cvCopyImage(trainingImage[i], inputImage);
	}
	cvResetImageROI(inputImage);

	cvNamedWindow("Sensor Dialog");
	cvResizeWindow("Sensor Dialog", 500, 120);
	int timeValue = 0;
	cvCreateTrackbar("Time", "Sensor Dialog", &timeValue, 10, TrackbarCallback);
	cvCreateTrackbar("Rain", "Sensor Dialog", &timeValue, 10, TrackbarCallback);
	cvCreateTrackbar("Wind", "Sensor Dialog", &timeValue, 10, TrackbarCallback);
	

	int updateIndex = 0;
	
	bool processing = true;
	cvNamedWindow("server");
	while(processing)
	{
		char ch = cvWaitKey(50);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}


		updateIndex++;
		if(updateIndex > 50)
		{
			int timeSensor = cvGetTrackbarPos("Time", "Sensor Dialog");
			int rainSensor = cvGetTrackbarPos("Rain", "Sensor Dialog");
			int windSensor = cvGetTrackbarPos("Wind", "Sensor Dialog");

			isUpdating = true;

			for(int i=0; i<OBJECT_COUNT; i++)
			{
//				sprintf(message, "canvas/%d.png", i);
				sprintf(message, "d:/tmp/canvas/%d.png", i);
				IplImage* tempImage = cvLoadImage(message);
				cvCopyImage(tempImage, canvasImage[i]);
				cvReleaseImage(&tempImage);
			}

			isUpdating = false;
			updateIndex = 0;
	

			IplImage* tempImage = cvCreateImage(cvSize(WIDTH1, HEIGHT1), IPL_DEPTH_8U, 3);
			for(int i=0; i<OBJECT_COUNT; i++)
			{
				cvSetImageROI(inputImage, cvRect((WIDTH1 + BORDER) * (i) + BORDER, BORDER, WIDTH1, HEIGHT1));
				cvCopyImage(inputImage, tempImage);

				if(timeSensor > 0)
					cvSmooth(canvasImage[i], canvasImage[i], CV_GAUSSIAN, timeSensor + timeSensor%2 + 1);

				if(rainSensor > 0)
					cvAddNoise(canvasImage[i], CV_NOISE_SALT_AND_PEPPER, 0.001 * rainSensor);

				if(windSensor > 0)
					cvAddNoise(canvasImage[i], CV_NOISE_GAUSSIAN, windSensor);



				windage::Utils::CompundImmersiveImage(canvasImage[i], tempImage, ALPHA_MASK_COLOR, 1.0);

				cvCopyImage(tempImage, inputImage);

				sprintf(message, "d:/tmp/canvas/%d.png", i);
				cvSaveImage(message, canvasImage[i]);
			}
			cvResetImageROI(inputImage);
			cvReleaseImage(&tempImage);

			for(int i=0; i<OBJECT_COUNT; i++)
			{
//				cvAddNoise(canvasImage[i], CV_NOISE_GAUSSIAN, 0.5);
				
			}
			

			log->log("update");
			log->logNewLine();
		}

		cvShowImage("server", inputImage);
	}

	cvDestroyAllWindows();
}
