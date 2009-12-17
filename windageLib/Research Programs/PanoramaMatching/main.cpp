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
const int ADAPTIVE_THRESHOLD_VALUE = 300;
const int THRESHOLD_STEP = 1;

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

double ratio = 0.0;
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

CvPoint CalucateCurrentPosition(windage::Calibration* calibration)
{
	CvPoint2D64f point1 = calibration->ConvertImage2World(0.0,	0.0,	0.0);
	CvPoint2D64f point3 = calibration->ConvertImage2World(WIDTH,HEIGHT, 0.0);

	CvPoint point;
	point.x = (point1.x + point3.x)/2;
	point.y = (point1.y + point3.y)/2;
	return point;
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
	const int W = 1920/ratio;
	const int H = 720/ratio;
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		setTarget = true;
		mousePosition.x = x;
		mousePosition.y = y;

		targetPosition.x = (mousePosition.x - (WIDTH - W)/2) * ratio;
		targetPosition.y = (mousePosition.y - (HEIGHT - H)) * ratio;
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

void DrawArrow(IplImage* image, CvPoint cur, CvPoint tar, windage::Calibration* calibration, CvScalar color = CV_RGB(255, 0, 0))
{
	int ARROW_WIDTH = 20;
	int BOARDER = 5;

	double dx = tar.x - cur.x;
	double dy = tar.y - cur.y;
	double length = sqrt(dx*dx + dy*dy);

	if(length > 100)
	{
		dx /= length;
		dy /= length;

		// fix right up
		tar.x = cur.x + dx * 100;
		tar.y = cur.y + dy * 100;

		double dir = atan2((double)tar.y - cur.y, (double)tar.x - cur.x);
		
		CvPoint pos0 = calibration->ConvertWorld2Image(cur.x, cur.y, 0.0);
		CvPoint pos1 = calibration->ConvertWorld2Image(tar.x, tar.y, 0.0);

		CvPoint pos2 = calibration->ConvertWorld2Image(tar.x + cos(120*CV_PI/180 +dir)*40, tar.y + sin(120*CV_PI/180 +dir)*40, 0.0);
		CvPoint pos3 = calibration->ConvertWorld2Image(tar.x + cos(240*CV_PI/180 +dir)*40, tar.y + sin(240*CV_PI/180 +dir)*40, 0.0);

		dx = (WIDTH/2) - pos0.x;
		dy = (HEIGHT/2) - pos0.y;

		pos0.x += dx;
		pos1.x += dx;
		pos2.x += dx;
		pos3.x += dx;

		pos0.y += dy;
		pos1.y += dy;
		pos2.y += dy;
		pos3.y += dy;	

		cvLine(image, pos0, pos1, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos1, pos2, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);
		cvLine(image, pos1, pos3, CV_RGB(255, 255, 255), ARROW_WIDTH + BOARDER);

		cvLine(image, pos0, pos1, color, ARROW_WIDTH);
		cvLine(image, pos1, pos2, color, ARROW_WIDTH);
		cvLine(image, pos1, pos3, color, ARROW_WIDTH);
	}
}


void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	// saving
	bool saving = false;
	CvVideoWriter* writer = NULL;

	char message[100];
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	
	// Tracker Initialize
	IplImage* trainingImage = cvLoadImage("panoramaImageRef.png", 0);
	IplImage* referenceImage = cvLoadImage("panoramaImageRef.png");
	IplImage* annotation = cvLoadImage("annotation.png");
	CompundImmersiveImage(annotation, referenceImage, CV_RGB(255, 0, 0), 1.0);

	ratio = (double)referenceImage->width / (double)inputImage->width + 0.5;
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
			position.y = (referenceImage->height - targetPosition.y) - referenceImage->height / 2.0;
			DrawCrossHair(inputImage, position, tracker->GetCameraParameter(), CV_RGB(0, 255, 0));

		}

		CvRect rect = cvRect((inputImage->width - resultImage->width)/2.0, inputImage->height - resultImage->height, resultImage->width, resultImage->height);
		cvSetImageROI(inputImage, rect);
		CompundImmersiveImage(resultImage, inputImage, CV_RGB(-1, -1, -1), 0.75);
		cvResetImageROI(inputImage);
		cvRectangle(inputImage, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width, rect.y+rect.height), CV_RGB(0, 0, 255), 3);

		if(setTarget)
		{
			CvPoint target;
			target.x = targetPosition.x - referenceImage->width / 2.0;
			target.y = (referenceImage->height - targetPosition.y) - referenceImage->height / 2.0;
			currentPosition = CalucateCurrentPosition(tracker->GetCameraParameter());
			DrawArrow(inputImage, currentPosition, target, tracker->GetCameraParameter(), CV_RGB(255, 0, 0));
		}

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
			saving = true;
			if(writer) cvReleaseVideoWriter(&writer);
			writer = cvCreateVideoWriter("saveimage\\capture.avi", CV_FOURCC_DEFAULT, 30, cvSize(inputImage->width, inputImage->height), 1);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}
		
		cvShowImage("result", inputImage);
	}

	if(writer) cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
