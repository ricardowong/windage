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
#include <vector>
#include <omp.h>

#include <highgui.h>
#include <windage.h>

//#include "PlaneEstimation.h"

#define RECTIFICATION
#define FLIP

const int WIDTH = 640;
const int HEIGHT = 480;
const char* templateFileName = "D:\\ImageSequence\\20090828\\capture%d.jpg";

const double intrinsic[] = {778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114};

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	IplImage* tempImage;
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);
	
	std::vector<windage::ModifiedSURFTracker*> trackerList;
	const char* message = "reference #%d";
	char resultMessage[100];

	cvNamedWindow("result image");

	bool isBreak = false;
	bool isProcessing = true;
	while(isProcessing)
	{
		fpslog->log("fps", fpslog->calculateFPS());
		fpslog->updateTickCount();
//		fpslog->logNewLine();

		// image grabbing
		log->updateTickCount();
		tempImage = cvQueryFrame(capture);
#ifdef RECTIFICATION
		calibration->Undistortion(tempImage, inputImage);
#else
		cvCopy(tempImage, inputImage);
#endif
#ifdef FLIP
		cvFlip(inputImage, inputImage);
#endif
		cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);
		log->log("capture", log->calculateProcessTime());

		// calcuate multiple tracking
		log->updateTickCount();

		#pragma omp parallel for schedule(dynamic)
		for(int i=0; i<trackerList.size(); i++)
		{
			trackerList[i]->UpdateCameraPose(grayImage);
//			trackerList[i]->DrawDebugInfo(inputImage);
			trackerList[i]->DrawInfomation(inputImage, 100.0);
			trackerList[i]->DrawOutLine(inputImage);

			#pragma omp critical
			{
				windage::Calibration* cameraPose = trackerList[i]->GetCameraParameter();
				CvPoint cameraPosition2D = cameraPose->ConvertWorld2Image(0, -10, 0);

				sprintf(resultMessage, message, i);
				windage::Utils::DrawTextToImage(inputImage, cameraPosition2D, resultMessage);
			}
		}
		log->log("tracking", log->calculateProcessTime());

		log->logNewLine();
		// draw result image
		cvShowImage("result image", inputImage);
		
		char ch;
		if(isBreak) ch = cvWaitKey();
		else		ch = cvWaitKey(1);
		switch(ch)
		{
		case 'a':
		case 'A':
			{
				std::cout << "attatch reference at current image" << std::endl;
				windage::ModifiedSURFTracker* tempTracker = new windage::ModifiedSURFTracker();
				tempTracker->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7], 45);
				tempTracker->RegistReferenceImage(grayImage, WIDTH, HEIGHT, 2.0, 4);
				tempTracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
				tempTracker->SetOpticalFlowRunning(true);
				trackerList.push_back(tempTracker);
			}
			break;
		case ' ':
			isBreak = !isBreak;
			break;
		case 'q':
		case 'Q':
			isProcessing = false;
			break;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}