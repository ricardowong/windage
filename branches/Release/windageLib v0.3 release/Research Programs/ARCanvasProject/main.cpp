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

#include "PGRCamera.h"

#define ADAPTIVE_THRESHOLD

const int OBJECT_COUNT = 6;
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

// Multiple canvas
windage::MultipleSURFTracker* multipleTracker;
IplImage* inputImage;
std::vector<IplImage*> canvasImage;
IplImage* resultCanvasImage;
int trackingIndex = -1;

CvScalar ALPHA_MASK_COLOR = CV_RGB(0, 255, 255);
CvScalar selectedColor = CV_RGB(255, 255, 255);
int size = 5;

bool isLButtonDown = false;
bool isRButtonDown = false;
CvPoint pt1, pt2;

bool isUpdating = false;

void MouseCallback(int event, int x, int y, int flags, void* param)
{
	if(isUpdating == false)
	{
		switch(event)
		{
		case CV_EVENT_LBUTTONDOWN:
			isLButtonDown = true;
			pt1.x = x;
			pt1.y = y;
			pt2.x = x;
			pt2.y = y;
			break;
		case CV_EVENT_LBUTTONUP:
			isLButtonDown = false;
			pt1.x = -1;
			pt1.y = -1;
			pt2.x = -1;
			pt2.y = -1;
			break;
		case CV_EVENT_RBUTTONDOWN:
			isRButtonDown = true;
			pt1.x = x;
			pt1.y = y;
			pt2.x = x;
			pt2.y = y;
			break;
		case CV_EVENT_RBUTTONUP:
			isRButtonDown = false;
			pt1.x = -1;
			pt1.y = -1;
			pt2.x = -1;
			pt2.y = -1;
			break;
		case CV_EVENT_MOUSEMOVE:
			if(isLButtonDown)
			{
				pt1.x = pt2.x;
				pt1.y = pt2.y;
				pt2.x = x;
				pt2.y = y;

				// draw
				if(trackingIndex >= 0)
				{
					int width = canvasImage[trackingIndex]->width;
					int height = canvasImage[trackingIndex]->height;
					CvPoint2D64f cPt1 = multipleTracker->GetCameraParameter(trackingIndex)->ConvertImage2World(pt1.x, pt1.y, 0.0);
					CvPoint2D64f cPt2 = multipleTracker->GetCameraParameter(trackingIndex)->ConvertImage2World(pt2.x, pt2.y, 0.0);
					cvLine(canvasImage[trackingIndex], cvPoint(cPt1.x + width/2, height/2 - cPt1.y), cvPoint(cPt2.x + width/2, height/2 - cPt2.y), selectedColor, size*2);
				}
			}
			else if(isRButtonDown)
			{
				pt1.x = pt2.x;
				pt1.y = pt2.y;
				pt2.x = x;
				pt2.y = y;

				// remove
				if(trackingIndex >= 0)
				{
					int width = canvasImage[trackingIndex]->width;
					int height = canvasImage[trackingIndex]->height;
					CvPoint2D64f cPt1 = multipleTracker->GetCameraParameter(trackingIndex)->ConvertImage2World(pt1.x, pt1.y, 0.0);
					CvPoint2D64f cPt2 = multipleTracker->GetCameraParameter(trackingIndex)->ConvertImage2World(pt2.x, pt2.y, 0.0);
					cvLine(canvasImage[trackingIndex], cvPoint(cPt1.x + width/2, height/2 - cPt1.y), cvPoint(cPt2.x + width/2, height/2 - cPt2.y), ALPHA_MASK_COLOR, size*2);
				}
			}

			cvCircle(inputImage, cvPoint(x, y), size, selectedColor, 2);

			break;
		}
	}
}

void WrapingCanvas(IplImage* canvas, IplImage* result, windage::Calibration* calibration)
{
//#pragma omp parallel for num_threads(2) schedule(dynamic)
	for(int y=0; y<result->height; y++)
	{
		for(int x=0; x<result->width; x++)
		{
			CvPoint2D64f pt = calibration->ConvertImage2World(x*2, y*2, 0.0);
			pt.x = canvas->width/2 + pt.x;
			pt.y = canvas->height/2 - pt.y;
			if( 0 <= pt.x && pt.x < canvas->width &&
				0 <= pt.y && pt.y < canvas->height)
			{
				CvScalar color = cvGet2D(canvas, pt.y, pt.x);
				cvSet2D(result, y, x, color);
			}
		}
	}
}

DWORD WINAPI WorkerThread(LPVOID)
{
	IplImage* buffer = cvCreateImage(cvSize(WIDTH/2, HEIGHT/2), IPL_DEPTH_8U, 3);
	while(true)
	{
		if(isUpdating == false)
		{
			cvSet(buffer, ALPHA_MASK_COLOR);
			for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
			{
				int matchedCount = multipleTracker->GetMatchedCount(i);
				if(matchedCount > FIND_FEATURE_COUNT)
				{
					WrapingCanvas(canvasImage[i], buffer, multipleTracker->GetCameraParameter(i));
				}
			}
			cvResize(buffer, resultCanvasImage);
//			cvCopyImage(buffer, resultCanvasImage);
		}
	}
	cvReleaseImage(&buffer);
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CPGRCamera* capture = new CPGRCamera();
	capture->open();
	capture->start();

	// saving
	bool saving = false;
	CvVideoWriter* writer = NULL;

	char message[500];
	inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* undistImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	// Multiple tracker Initialize
	std::vector<IplImage*> trainingImage;
	resultCanvasImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 3);
	for(int i=1; i<=OBJECT_COUNT; i++)
	{
		sprintf(message, "reference%d_320.png", i);
		trainingImage.push_back(cvLoadImage(message, 0));

		canvasImage.push_back(cvCreateImage(cvSize(WIDTH/2, HEIGHT/2), IPL_DEPTH_8U, 3));
		cvSet(canvasImage[i-1], ALPHA_MASK_COLOR);
	}

	multipleTracker = new windage::MultipleSURFTracker();
	multipleTracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	multipleTracker->InitializeOpticalFlow(WIDTH, HEIGHT, cvSize(8, 8), 3);
	multipleTracker->SetDetectIntervalTime(1.0/1.0);
	multipleTracker->SetPoseEstimationMethod(windage::PROSAC);
	multipleTracker->SetOutlinerRemove(true);
	multipleTracker->SetRefinement(true);
	multipleTracker->SetFeatureExtractThreshold(30);
	for(int i=0; i<trainingImage.size(); i++)
	{
		std::cout << "attatch reference image #" << i << std::endl;
		multipleTracker->AttatchReferenceImage(trainingImage[i], trainingImage[i]->width, trainingImage[i]->height, 8.0, 8);
	}

	DWORD dwID;
	CreateThread(NULL, 0, WorkerThread, NULL, 0, &dwID);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
	const int THRESHOLD_STEP = 1;

	IplImage* grabFrame = NULL;

	int updateIndex = 0;
	
	bool processing = true;
	cvNamedWindow("result");
	cvSetMouseCallback("result", MouseCallback);

	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		capture->update();
		IplImage* grabFrame = capture->GetIPLImage();
		cvCvtColor(grabFrame, undistImage, CV_BGRA2BGR);
		calibration->Undistortion( undistImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		multipleTracker->SetFeatureExtractThreshold(fastThreshold);
		multipleTracker->UpdateCameraPose(grayImage);

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();

		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		int featureCount = multipleTracker->GetFeatureCount();
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif
		
		// draw tracking result
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			int matchedCount = multipleTracker->GetMatchedCount(i);
			if(matchedCount > FIND_FEATURE_COUNT)
			{
//				multipleTracker->DrawOutLine(inputImage, i, true);
//				multipleTracker->DrawInfomation(inputImage, i, 50.0);
//				multipleTracker->DrawDebugInfo(inputImage);

				CvPoint center = multipleTracker->GetCameraParameter(i)->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				center.x += 10;
				center.y += 10;
				sprintf(message, "Reference #%d", i);
				windage::Utils::DrawTextToImage(inputImage, center, message);

				trackingIndex = i;
				
			}
		}
		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count ");
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
		for(int i=0; i<OBJECT_COUNT; i++)
		{
			sprintf(message, "#%d:%d ", i, multipleTracker->GetMatchedCount(i));
			windage::Utils::DrawTextToImage(inputImage, cvPoint(160 + 65*i, 70), message);
		}

		windage::Utils::CompundImmersiveImage(resultCanvasImage, inputImage, ALPHA_MASK_COLOR, 1.0);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case '1':
			selectedColor = CV_RGB(255, 255, 255);
			break;
		case '2':
			selectedColor = CV_RGB(255, 0, 0);
			break;
		case '3':
			selectedColor = CV_RGB(0, 255, 0);
			break;
		case '4':
			selectedColor = CV_RGB(0, 0, 255);
			break;
		case '>':
		case '.':
			size++;
			break;
		case '<':
		case ',':
			size--;
			break;
		case 's':
		case 'S':
			saving = true;
			{
				if(writer) cvReleaseVideoWriter(&writer);
				CvRNG rng = cvRNG(cvGetTickCount());
				sprintf(message, "saveimage\\capture_%d.avi", cvRandInt(&rng)%10);
				writer = cvCreateVideoWriter(message, CV_FOURCC_DEFAULT, 30, cvSize(inputImage->width, inputImage->height), 1);
			}
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		updateIndex++;
		if(updateIndex > 30)
		{
			isUpdating = true;

			for(int i=0; i<OBJECT_COUNT; i++)
			{
//				sprintf(message, "canvas/%d.png", i);
				sprintf(message, "d:/tmp/canvas/%d.png", i);
				IplImage* tempImage = cvLoadImage(message);
				IplImage* tempImage2 = cvCloneImage(canvasImage[i]);

				windage::Utils::CompundImmersiveImage(tempImage2, tempImage, ALPHA_MASK_COLOR, 1.0);
				cvSaveImage(message, tempImage);

				cvCopyImage(tempImage, canvasImage[i]);

				cvReleaseImage(&tempImage);
				cvReleaseImage(&tempImage2);
			}

			isUpdating = false;
			updateIndex = 0;
		}

		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}
		cvShowImage("result", inputImage);
	}

	if(writer) cvReleaseVideoWriter(&writer);
	capture->stop();
	cvDestroyAllWindows();
}
