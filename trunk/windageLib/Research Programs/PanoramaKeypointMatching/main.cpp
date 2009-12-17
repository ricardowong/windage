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

#define FLIP
#define RECTIFICATION
#define ADAPTIVE_THRESHOLD

//#define USE_IMAGE_SEQUENCE

const int FIND_FEATURE_COUNT = 10;

const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 20;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

std::vector<CvPoint> panoramaPositions;

const int WIDTH = 640;
const int HEIGHT = 480;
int realWidth = 0;
int realHeight = 0;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, refImage->width, refImage->height, 2.0, 4);
	tracker->SetPoseEstimationMethod(windage::LMEDS);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->SetRefinement(true);
//	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void DrawTarget(IplImage* image, windage::Calibration* calibration, CvPoint center, double ratio=0.5)
{
	CvPoint2D64f point1 = calibration->ConvertImage2World(0.0,	0.0,	0.0);
	CvPoint2D64f point2 = calibration->ConvertImage2World(WIDTH,0.0,	0.0);
	CvPoint2D64f point3 = calibration->ConvertImage2World(WIDTH,HEIGHT, 0.0);
	CvPoint2D64f point4 = calibration->ConvertImage2World(0.0,	HEIGHT, 0.0);

	point1.x *= ratio;
	point2.x *= ratio;
	point3.x *= ratio;
	point4.x *= ratio;

	point1.y *= ratio;
	point2.y *= ratio;
	point3.y *= ratio;
	point4.y *= ratio;

	point1.x += image->width/2.0;
	point2.x += image->width/2.0;
	point3.x += image->width/2.0;
	point4.x += image->width/2.0;

	point1.y += image->height/2.0;
	point2.y += image->height/2.0;
	point3.y += image->height/2.0;
	point4.y += image->height/2.0;

	point1.y = image->height - point1.y;
	point2.y = image->height - point2.y;
	point3.y = image->height - point3.y;
	point4.y = image->height - point4.y;

	cvLine(image, cvPoint(point1.x, point1.y), cvPoint(point2.x, point2.y), CV_RGB(255, 255, 0), 3);
	cvLine(image, cvPoint(point2.x, point2.y), cvPoint(point3.x, point3.y), CV_RGB(255, 255, 0), 3);
	cvLine(image, cvPoint(point3.x, point3.y), cvPoint(point4.x, point4.y), CV_RGB(255, 255, 0), 3);
	cvLine(image, cvPoint(point4.x, point4.y), cvPoint(point1.x, point1.y), CV_RGB(255, 255, 0), 3);

	// draw center poisition
	center.x *= ratio;
	center.y *= ratio;
	cvCircle(image, center, 5, CV_RGB(255, 0, 0), 2);
}

CvPoint GetAverageFeaturePosition(std::vector<SURFDesciription>* points)
{
	CvPoint point = cvPoint(0, 0);
	int count = points->size();
	if(count > 0)
	{
		for(int i=0; i<count; i++)
		{
			point.x += (*points)[i].point.x;
			point.y += (*points)[i].point.y;
		}

		point.x /= count;
		point.y /= count;
	}

	point.x += realWidth/2;
	point.y += realHeight/2;
	point.y = realHeight - point.y;
	return point;
}

int GetIndexPosition(CvPoint point)
{
	double minDistance = 999999999;
	int minIndex = -1;

	for(int i=0; i<panoramaPositions.size(); i++)
	{
		double dx = (point.x - panoramaPositions[i].x);
		double dy = (point.y - panoramaPositions[i].y);
		double distance = sqrt(dx*dx + dy*dy);

		if(minDistance > distance)
		{
			minDistance = distance;
			minIndex = i;
		}
	}

	return minIndex;
}

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	char message[100];
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	
	// Tracker Initialize
	double ratio = 2.0;
	IplImage* trainingImage = cvLoadImage("panoramaImageRef.png", 0);
	IplImage* referenceImage = cvLoadImage("panoramaImageRef.png");
	IplImage* resultImage = cvCreateImage(cvSize(referenceImage->width/ratio, referenceImage->height/ratio), IPL_DEPTH_8U, 3);
	windage::ModifiedSURFTracker* tracker = CreateTracker(trainingImage, 0);

	realWidth = trainingImage->width;
	realHeight = trainingImage->height;

	// add keypoint image
	std::vector<windage::ModifiedSURFTracker*> keypointTracker;
	const int KEYPOINT_WIDTH = 640/2;
	const int KEYPOINT_HEIGHT = 480/2;
	for(int y=0; y<(referenceImage->height / KEYPOINT_HEIGHT) - 1; y++)
	{
		for(int x=0; x<(referenceImage->width / KEYPOINT_WIDTH) - 1; x++)
		{
			CvRect rect = cvRect(KEYPOINT_WIDTH * x, KEYPOINT_HEIGHT * y, KEYPOINT_WIDTH * 2, KEYPOINT_HEIGHT * 2);
			std::cout << rect.x << ", " << rect.y << ", " << rect.width << ", " << rect.height << std::endl;
			panoramaPositions.push_back(cvPoint(rect.x + rect.width/2, rect.y + rect.height/2));

			cvSetImageROI(trainingImage, rect);
			keypointTracker.push_back(CreateTracker(trainingImage, x));
		}
	}
	cvResetImageROI(trainingImage);
	IplImage* keypointImage = cvCreateImage(cvSize(KEYPOINT_WIDTH*2, KEYPOINT_HEIGHT*2), IPL_DEPTH_8U, 3);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	IplImage* grabFrame = NULL;

	bool processing = true;
	cvNamedWindow("result");
	cvNamedWindow("panorama");
	cvNamedWindow("keypoint");
	while(processing)
	{
		// camera frame grabbing and convert to gray and undistortion
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, grabFrame);
		cvResize(grabFrame, tempImage);
		calibration->Undistortion(tempImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();

		tracker->SetFeatureExtractThreshold(fastThreshold);
		tracker->UpdateCameraPose(grayImage);
		int featureCount = tracker->GetFeatureCount();
		int matchingCount = tracker->GetMatchedCount();

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();
			
		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif
		
		int referenceIndex = -1;
		CvPoint referencePoint = cvPoint(0, 0);
		// draw tracking result
		if(featureCount > FIND_FEATURE_COUNT)
		{
//			tracker->DrawOutLine(inputImage, true);
//			tracker->DrawInfomation(inputImage, 100.0);
//			tracker->DrawDebugInfo(inputImage);

			referencePoint = GetAverageFeaturePosition(tracker->GetMatchedReferencePoints());
			referenceIndex = GetIndexPosition(referencePoint);

			keypointTracker[referenceIndex]->UpdateCameraPose(grayImage);
			
			cvSetImageROI(referenceImage, cvRect(panoramaPositions[referenceIndex].x - KEYPOINT_WIDTH, panoramaPositions[referenceIndex].y - KEYPOINT_HEIGHT, KEYPOINT_WIDTH*2, KEYPOINT_HEIGHT*2));
			cvCopyImage(referenceImage, keypointImage);
			cvResetImageROI(referenceImage);

			keypointTracker[referenceIndex]->DrawInfomation(keypointImage);
			keypointTracker[referenceIndex]->DrawDebugInfo(keypointImage);
		}

		// draw panorama
		cvResize(referenceImage, resultImage);
		DrawTarget(resultImage, tracker->GetCameraParameter(), referencePoint, 1.0/2.0);

		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count : %d", matchingCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);

		sprintf(message, "matched point : %d, %d (%d)", referencePoint.x, referencePoint.y, referenceIndex);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 90), message);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 's':
		case 'S':
			cvSaveImage("matching.png", resultImage);
			cvSaveImage("result.png", inputImage);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("keypoint", keypointImage);
		cvShowImage("panorama", resultImage);
		cvShowImage("result", inputImage);
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
