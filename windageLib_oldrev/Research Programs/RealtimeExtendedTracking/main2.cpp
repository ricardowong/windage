#define RUNNING
#ifdef RUNNING

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

const int PATCH_WIDTH = 300;
const int PATCH_HEIGHT = 300;

const double intrinsic[] = {778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114};

CvPoint mousePoint;

bool updateTracker = false;
CvPoint imagePoint;
void MouseEvent( int mevent, int x, int y, int flags, void* param )
{
   switch(mevent)
   {
   case CV_EVENT_LBUTTONDOWN:
	   mousePoint.x = x;
	   mousePoint.y = y;
	   updateTracker = true;
	   break;
   case CV_EVENT_LBUTTONUP:
	   break;
   case CV_EVENT_RBUTTONDOWN:
	   break;
   case CV_EVENT_RBUTTONUP:
	   break;
   case CV_EVENT_MOUSEMOVE:
	   mousePoint.x = x;
	   mousePoint.y = y;
	   break;
   }
}

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

	IplImage* patchImage = cvCreateImage(cvSize(PATCH_WIDTH, PATCH_HEIGHT), IPL_DEPTH_8U, 1);

	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	windage::Calibration* savedCalibration = new windage::Calibration();
	savedCalibration->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7]);
	savedCalibration->InitUndistortionMap(WIDTH, HEIGHT);


	std::vector<windage::Matrix4> relationT;
	std::vector<windage::ModifiedSURFTracker*> trackerList;
	const char* message = "reference #%d";
	char resultMessage[100];

	cvNamedWindow("result image");
	cvSetMouseCallback("result image",MouseEvent);

	bool isBreak = false;
	bool isProcessing = true;
	while(isProcessing)
	{
//		fpslog->logNewLine();

		fpslog->updateTickCount();

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

//		#pragma omp parallel for schedule(dynamic)
		for(int i=0; i<trackerList.size(); i++)
		{
			if(trackerList[i]->UpdateCameraPose(grayImage) > 0.0)
			{
//				trackerList[i]->DrawDebugInfo(inputImage);

				trackerList[i]->DrawOutLine(inputImage);
				trackerList[i]->DrawInfomation(inputImage, 100.0);

				#pragma omp critical
				{
					windage::Calibration* cameraPose = trackerList[i]->GetCameraParameter();
					CvPoint cameraPosition2D = cameraPose->ConvertWorld2Image(0, -20, 0);

					sprintf(resultMessage, message, i);
					windage::Utils::DrawTextToImage(inputImage, cameraPosition2D, resultMessage);
				}
			}
			else
			{
				int prevIndex = i-1;
				if(prevIndex >= 0 && relationT.size() == trackerList.size())
				{
					CvMat* extrinsic2 = trackerList[i]->GetCameraParameter()->GetExtrinsicMatrix();
					CvMat* extrinsic1 = trackerList[prevIndex]->GetCameraParameter()->GetExtrinsicMatrix();

					for(int y=0; y<4; y++)
					{
						for(int x=0; x<4; x++)
						{
							cvSetReal2D(extrinsic2, y, x, cvGetReal2D(extrinsic1, y, x) + relationT[prevIndex].m[y][x]);
						}
					}

					trackerList[i]->DrawOutLine(inputImage);
					trackerList[i]->DrawInfomation(inputImage, 100.0);
				}
			}
		}
		log->log("tracking", log->calculateProcessTime());
		log->logNewLine();

		sprintf(resultMessage, "fps : %lf", fpslog->calculateFPS());

		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), resultMessage);
/*
		if(trackerList.size() > 0)
		{
			cvCircle(inputImage, imagePoint, 10, CV_RGB(255, 0, 0));
			CvPoint2D64f worldPoint1 = savedCalibration->ConvertImage2World(imagePoint.x, imagePoint.y, 0.0);
			CvPoint2D64f worldPoint2 = savedCalibration->ConvertImage2World(imagePoint.x, imagePoint.y, 1000.0);

			CvPoint point1 = trackerList[0]->GetCameraParameter()->ConvertWorld2Image(worldPoint1.x, worldPoint1.y, 0.0);
			CvPoint point2 = trackerList[0]->GetCameraParameter()->ConvertWorld2Image(worldPoint2.x, worldPoint2.y, 1000.0);

			double dx = point2.x - point1.x;
			double dy = point2.y - point1.y;

			double a = dy / dx;
			double b = point1.y - a * point1.x;
			
			CvPoint left = cvPoint(0, b);
			CvPoint right = cvPoint(WIDTH, a*WIDTH + b);

			cvLine(inputImage, left, right, CV_RGB(0, 255, 0), 3);
//			cvLine(inputImage, trackerList[0]->GetCameraParameter()->ConvertWorld2Image(worldPoint1.x, worldPoint1.y, 0.0), trackerList[0]->GetCameraParameter()->ConvertWorld2Image(savedCalibration->GetCameraPosition().val[0], savedCalibration->GetCameraPosition().val[1], savedCalibration->GetCameraPosition().val[2]), CV_RGB(0, 0, 255), 3);
		}

		if(updateTracker)
		{
			savedCalibration->SetExtrinsicMatrix(trackerList[0]->GetCameraParameter()->GetExtrinsicMatrix());
			updateTracker = false;
		}
*/

		if(updateTracker)
		{
			cvSetImageROI(grayImage, cvRect(mousePoint.x - PATCH_WIDTH/2, mousePoint.y - PATCH_HEIGHT/2, PATCH_WIDTH, PATCH_HEIGHT));
			cvCopy(grayImage, patchImage);
			cvResetImageROI(grayImage);

			std::cout << "attatch reference at current patch image" << std::endl;
			windage::ModifiedSURFTracker* tempTracker = new windage::ModifiedSURFTracker();
			tempTracker->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7], 30);
			tempTracker->RegistReferenceImage(patchImage, PATCH_WIDTH, PATCH_HEIGHT, 8.0, 8);
			tempTracker->InitializeOpticalFlow(WIDTH, HEIGHT, 15, cvSize(15, 15), 3);
			tempTracker->SetOpticalFlowRunning(true);
			trackerList.push_back(tempTracker);

			tempTracker->SetFeatureExtractTreshold(45);

			updateTracker = false;
		}
		if(mousePoint.x > 0)
			cvRectangle(inputImage, cvPoint(mousePoint.x - PATCH_WIDTH/2, mousePoint.y - PATCH_HEIGHT/2), cvPoint(mousePoint.x + PATCH_WIDTH/2, mousePoint.y + PATCH_HEIGHT/2), CV_RGB(255, 0, 0), 3);
		mousePoint.x = -1;
		

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
				tempTracker->Initialize(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5], intrinsic[6], intrinsic[7], 30);
				tempTracker->RegistReferenceImage(grayImage, WIDTH, HEIGHT, 4.0, 8);
				tempTracker->InitializeOpticalFlow(WIDTH, HEIGHT, 15, cvSize(15, 15), 3);
				tempTracker->SetOpticalFlowRunning(true);
				trackerList.push_back(tempTracker);

				tempTracker->SetFeatureExtractTreshold(50);
			}
			break;
		case 'c':
		case 'C':
			{
				windage::Matrix4 tempMatrix;

				relationT.clear();
				for(int i=0; i<trackerList.size(); i++)
				{
					int nextIndex = (i+1)%trackerList.size();

					CvMat* tempExtrinsic1 = trackerList[i]->GetCameraParameter()->GetExtrinsicMatrix();
					CvMat* tempExtrinsic2 = trackerList[nextIndex]->GetCameraParameter()->GetExtrinsicMatrix();

					for(int y=0; y<4; y++)
					{
						for(int x=0; x<4; x++)
						{
							tempMatrix.m[y][x] = cvGetReal2D(tempExtrinsic2, y, x) - cvGetReal2D(tempExtrinsic1, y, x);
						}
					}

					relationT.push_back(tempMatrix);
				}
			}
			break;
		case 'r':
		case 'R':
			{
				if(trackerList.size() > 0)
				{
					std::cout << "remove reference at last image" << std::endl;
					windage::ModifiedSURFTracker* tempTracker = trackerList[trackerList.size()-1];
					delete tempTracker;
					trackerList.erase(trackerList.end() - 1);
				}
			}
			break;
		case 's':
		case 'S':
			if(trackerList.size() > 0)
				savedCalibration->SetExtrinsicMatrix(trackerList[0]->GetCameraParameter()->GetExtrinsicMatrix());
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
#endif