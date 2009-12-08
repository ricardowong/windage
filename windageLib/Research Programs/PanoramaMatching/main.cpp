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

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

bool setTarget = false;
int state = 0;
CvPoint mousePosition;
CvPoint targetPosition;
CvPoint currentPosition;

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, refImage->width, refImage->height, 4.0, 8);
	tracker->SetPoseEstimationMethod(windage::RANSAC);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 3, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->SetRefinement(true);
//	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void DrawTarget(IplImage* image, windage::Calibration* calibration, double ratio=0.5)
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
}

bool SameColor(CvScalar color1, CvScalar color2)
{
	if( color1.val[0] == color2.val[0] &&
		color1.val[1] == color2.val[1] &&
		color1.val[2] == color2.val[2] &&
		color1.val[3] == color2.val[3])
		return true;
	else
		return false;
}

bool CompundImmersiveImage(IplImage* src, IplImage* dst, CvScalar maskColor, double alpha)
{
	for(int y=0; y<src->height; y++)
	{
		for(int x=0; x<src->width; x++)
		{
			CvScalar color = cvGet2D(src, y, x);
			if(!SameColor(color, maskColor))
			{
				CvScalar dstColor = cvGet2D(dst, y, x);
				for(int i=0; i<4; i++)
					color.val[i] = color.val[i] * alpha + dstColor.val[i] * (1.0 - alpha);
				cvSet2D(dst, y, x, color);
			}
		}
	}
	return true;
}

void MouseEvent(int event, int x, int y, int flags, void* param)
{
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		mousePosition.x = x;
		mousePosition.y = y;

		setTarget = true;
		if( x < WIDTH / 3)
		{
			state = 1;
			targetPosition.x = WIDTH /2;
		}
		else if(WIDTH / 3 <= x && x < WIDTH * 2 / 3)
		{
			state = 2;
			targetPosition.x = WIDTH /2 + WIDTH;
		}
		else
		{
			state = 3;
			targetPosition.x = WIDTH /2 + WIDTH * 2;
		}
		targetPosition.y = HEIGHT/2 - 10;
		break;
	}
}

void DrawCrossHair(IplImage* image, CvPoint position, windage::Calibration* calibration, CvScalar color = CV_RGB(255, 0, 0))
{
	int CIRCLE_WIDTH = 5;
	int BOARDER = 5;

	if(calibration == NULL)
	{
		cvCircle(image, position, 30, CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(position.x-40, position.y), cvPoint(position.x-15, position.y), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(position.x+15, position.y), cvPoint(position.x+40, position.y), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(position.x, position.y-40), cvPoint(position.x, position.y-15), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(position.x, position.y+15), cvPoint(position.x, position.y+40), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);

		cvCircle(image, position, 30, color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(position.x-40, position.y), cvPoint(position.x-15, position.y), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(position.x+15, position.y), cvPoint(position.x+40, position.y), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(position.x, position.y-40), cvPoint(position.x, position.y-15), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(position.x, position.y+15), cvPoint(position.x, position.y+40), color, CIRCLE_WIDTH);
	}
	else
	{
		CvPoint target = calibration->ConvertWorld2Image(position.x, position.y, 0.0);

		cvCircle(image, target, 30, CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(target.x-40, target.y), cvPoint(target.x-15, target.y), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(target.x+15, target.y), cvPoint(target.x+40, target.y), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(target.x, target.y-40), cvPoint(target.x, target.y-15), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);
		cvLine(image, cvPoint(target.x, target.y+15), cvPoint(target.x, target.y+40), CV_RGB(255, 255, 255), CIRCLE_WIDTH+BOARDER);

		cvCircle(image, target, 30, color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(target.x-40, target.y), cvPoint(target.x-15, target.y), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(target.x+15, target.y), cvPoint(target.x+40, target.y), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(target.x, target.y-40), cvPoint(target.x, target.y-15), color, CIRCLE_WIDTH);
		cvLine(image, cvPoint(target.x, target.y+15), cvPoint(target.x, target.y+40), color, CIRCLE_WIDTH);
	}
}

void DrawArrow(IplImage* image, CvPoint position, windage::Calibration* calibration, bool isLeft, CvScalar color = CV_RGB(255, 0, 0))
{
	int ARROW_WIDTH = 20;
	int BOARDER = 5;
	if(isLeft)
	{
		CvPoint pos0 = calibration->ConvertWorld2Image(position.x-40, position.y, 0.0);
		CvPoint pos1 = calibration->ConvertWorld2Image(position.x+40, position.y, 0.0);
		CvPoint pos2 = calibration->ConvertWorld2Image(position.x-20, position.y+20, 0.0);
		CvPoint pos3 = calibration->ConvertWorld2Image(position.x-20, position.y-20, 0.0);

		cvLine(image, pos0, pos1, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos0, pos2, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos0, pos3, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);

		cvLine(image, pos0, pos1, color, ARROW_WIDTH);
		cvLine(image, pos0, pos2, color, ARROW_WIDTH);
		cvLine(image, pos0, pos3, color, ARROW_WIDTH);
	}
	else
	{
		CvPoint pos0 = calibration->ConvertWorld2Image(position.x+40, position.y, 0.0);
		CvPoint pos1 = calibration->ConvertWorld2Image(position.x-40, position.y, 0.0);
		CvPoint pos2 = calibration->ConvertWorld2Image(position.x+20, position.y+20, 0.0);
		CvPoint pos3 = calibration->ConvertWorld2Image(position.x+20, position.y-20, 0.0);

		cvLine(image, pos0, pos1, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos0, pos2, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos0, pos3, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);

		cvLine(image, pos0, pos1, color, ARROW_WIDTH);
		cvLine(image, pos0, pos2, color, ARROW_WIDTH);
		cvLine(image, pos0, pos3, color, ARROW_WIDTH);
	}
}

void DrawArrowAll(IplImage* image, windage::Calibration* calibration, CvScalar color = CV_RGB(255, 0, 0))
{
	CvPoint pos;
	pos.y = HEIGHT/2 - 10 - (HEIGHT/4);

	for(int y=0; y<3; y++)
	{
		pos.y = HEIGHT/2 - 10 - (HEIGHT/4) * y;
		switch(state)
		{
		case 1:
			pos.x = WIDTH/2 + WIDTH*1 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);

			pos.x = WIDTH/2 + WIDTH*1.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);

			pos.x = WIDTH/2 + WIDTH*2 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);

			pos.x = WIDTH/2 + WIDTH*2.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);
			break;
		case 2:
			pos.x = WIDTH/2 + WIDTH*0 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);

			pos.x = WIDTH/2 + WIDTH*0.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);

			pos.x = WIDTH/2 + WIDTH*1.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);

			pos.x = WIDTH/2 + WIDTH*2 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, true, color);
			break;
		case 3:
			pos.x = WIDTH/2 + WIDTH*0 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);

			pos.x = WIDTH/2 + WIDTH*0.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);

			pos.x = WIDTH/2 + WIDTH*1 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);

			pos.x = WIDTH/2 + WIDTH*1.5 - (WIDTH*3/2);
			DrawArrow(image, pos, calibration, false, color);
			break;
		}
	}
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
	IplImage* trainingImage = cvLoadImage("panoramaImageRef.png", 0);
	IplImage* referenceImage = cvLoadImage("panoramaImageRef.png");
	IplImage* annotation = cvLoadImage("annotation.png");
	CompundImmersiveImage(annotation, referenceImage, CV_RGB(255, 0, 0), 1.0);

	double ratio = (double)referenceImage->width / (double)inputImage->width + 0.2;
	IplImage* resultImage = cvCreateImage(cvSize(referenceImage->width/ratio, referenceImage->height/ratio), IPL_DEPTH_8U, 3);
	windage::ModifiedSURFTracker* tracker = CreateTracker(trainingImage, 0);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	IplImage* grabFrame = NULL;

	bool processing = true;

	cvNamedWindow("result");
	cvSetMouseCallback("result", MouseEvent, NULL);
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
		
		// draw tracking result
		if(featureCount > FIND_FEATURE_COUNT)
		{
//			tracker->DrawOutLine(inputImage, true);
//			tracker->DrawInfomation(inputImage, 100.0);
//			tracker->DrawDebugInfo(inputImage);
		}

		// draw panorama
		cvResize(referenceImage, resultImage);
		DrawTarget(resultImage, tracker->GetCameraParameter(), 1.0/ratio);

		if(setTarget)
		{
			CvPoint position;
			position.x = targetPosition.x * 1.0/ratio;
			position.y = targetPosition.y * 1.0/ratio;
			DrawCrossHair(resultImage, position, NULL, CV_RGB(0, 255, 0));

			position.x = targetPosition.x - referenceImage->width / 2.0;
			position.y = targetPosition.y - referenceImage->height / 6.0;
			DrawCrossHair(inputImage, position, tracker->GetCameraParameter(), CV_RGB(0, 255, 0));

			DrawArrowAll(inputImage, tracker->GetCameraParameter(), CV_RGB(255, 0, 0));
		}

		cvSetImageROI(inputImage, cvRect((inputImage->width - resultImage->width)/2.0, inputImage->height - resultImage->height, resultImage->width, resultImage->height));
		CompundImmersiveImage(resultImage, inputImage, CV_RGB(-1, -1, -1), 0.75);
		cvResetImageROI(inputImage);

		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count : %d", matchingCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);

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

		
		cvShowImage("result", inputImage);
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
