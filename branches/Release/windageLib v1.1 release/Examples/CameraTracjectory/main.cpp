/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo (wwoo@gist.ac.kr)
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

#include <gl/glut.h>
#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/OpenGLRenderer.h"

#define USE_TEMPLATE_IMAEG 1
const char* TEMPLATE_IMAGE = "reference.png";

const double SCALE_FACTOR = 4.0;
const int SCALE_STEP = 8;

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

const double VIRTUAL_CAMERA_DISTANCE = 800.0;

windage::Logger* logging;
double fps;
const int FPS_UPDATE_STEP = 10;
int fpsStep = 0;

const int FEATURE_COUNT = WIDTH*2;
int keypointCount = 0;
double threshold = 50.0;

CvCapture* capture = NULL;
IplImage* resizeImage = NULL;
IplImage* grayImage = NULL;
IplImage* resultImage = NULL;

windage::Frameworks::PlanarObjectTracking* tracker = NULL;
OpenGLRenderer* renderer = NULL;
double angle = 0.0;

bool flip = true;

windage::Frameworks::PlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::PlanarObjectTracking* tracker = new windage::Frameworks::PlanarObjectTracking();

	windage::Calibration* calibration = new windage::Calibration();
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::WSURFdetector();
	windage::Algorithms::SearchTree* searchtree = new windage::Algorithms::FLANNtree();
	windage::Algorithms::OpticalFlow* opticalflow = new windage::Algorithms::OpticalFlow();
	windage::Algorithms::HomographyEstimator* estimator = new windage::Algorithms::ProSACestimator();
	windage::Algorithms::OutlierChecker* checker = new windage::Algorithms::OutlierChecker();
	windage::Algorithms::HomographyRefiner* refiner = new windage::Algorithms::LMmethod();
	windage::Algorithms::KalmanFilter* filter = new windage::Algorithms::KalmanFilter();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(8, 8), 3);
	estimator->SetReprojectionError(10.0);
	checker->SetReprojectionError(10.0);
	refiner->SetMaxIteration(10);

	tracker->AttatchCalibration(calibration);
	tracker->AttatchDetetor(detector);
	tracker->AttatchMatcher(searchtree);
	tracker->AttatchTracker(opticalflow);
	tracker->AttatchEstimator(estimator);
	tracker->AttatchChecker(checker);
	tracker->AttatchRefiner(refiner);
//	tracker->AttatchFilter(filter);

	tracker->SetDitectionRatio(30);
	tracker->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	return tracker;
}

void TrainingRefereneImage(windage::Frameworks::PlanarObjectTracking* tracker, IplImage* refImage)
{
	tracker->GetDetector()->SetThreshold(30.0);

	tracker->AttatchReferenceImage(refImage);
	tracker->TrainingReference(SCALE_FACTOR, SCALE_STEP);

	tracker->GetDetector()->SetThreshold(threshold);
}

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 's':
	case 'S':
	case ' ':
		renderer->AttatchReference(resizeImage);
		TrainingRefereneImage(tracker, grayImage);
		break;
	case 'f':
	case 'F':
		flip = !flip;
		break;
	case 'q':
	case 'Q':
		if(capture) cvReleaseCapture(&capture);
		cvDestroyAllWindows();
		exit(0);
		break;
	}
}

void idle(void)
{
	angle += 1.0;
	if(angle >= 360.0)
		angle = 0.0;
	glutPostRedisplay();
}

void display()
{
	// capture from camera
	IplImage* grabImage = cvRetrieveFrame(capture);
	if(flip)
		cvFlip(grabImage, grabImage);

	cvResize(grabImage, resizeImage);
	cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
	cvCopyImage(resizeImage, resultImage);

	// update camera pose
	tracker->UpdateCamerapose(grayImage);
	tracker->DrawDebugInfo(resultImage);
	tracker->DrawOutLine(resultImage, true);
	tracker->GetCameraParameter()->DrawInfomation(resultImage, WIDTH/4);
	
	int matchingCount = tracker->GetMatchingCount();

	// adaptive threshold
	int localcount = tracker->GetDetector()->GetKeypointsCount();
	if(keypointCount != localcount) // if updated
	{
		if(localcount > FEATURE_COUNT)
			threshold += 1;
		if(localcount < FEATURE_COUNT)
			threshold -= 1;
		keypointCount = localcount;
		tracker->GetDetector()->SetThreshold(threshold);
	}

	// calculate fps
    fpsStep++;
    if(fpsStep >= FPS_UPDATE_STEP)
    {
		fps = logging->calculateFPS()*(double)FPS_UPDATE_STEP;
		logging->updateTickCount();
		fpsStep = 0;
    }

	char message[100];
    sprintf_s(message, "FPS : %.2lf", fps);
    windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
	sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", keypointCount, threshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

	sprintf_s(message, "Press 'Space' to track the current image");
	windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
	sprintf_s(message, "Press 'F' to flip image");
	windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-25), 0.5, message);
	cvShowImage("tracking information window", resultImage);

	// clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw virtual object
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	double radian = angle * CV_PI / 180.0;
	double dx = sin(radian) * VIRTUAL_CAMERA_DISTANCE;
	double dy = cos(radian) * VIRTUAL_CAMERA_DISTANCE;
	gluLookAt(dx, dy, 2000, 0.0, 0.0, 600.0, 0.0, 0.0, 1.0);

	glPushMatrix();
	{
		// draw reference image & coordinate
		renderer->DrawReference((double)WIDTH, (double)HEIGHT);
		renderer->DrawAxis((double)WIDTH / 4.0);

		// draw camera image & position
		renderer->DrawCamera(tracker->GetCameraParameter(), resizeImage);
	}
	glPopMatrix();

	glutSwapBuffers();
}

void main()
{
	// connect camera
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	if(!capture)
	{
		std::cout << "can not connect any camera" << std::endl;
		exit(0);
	}

	cvNamedWindow("tracking information window");

	resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	logging = new windage::Logger(&std::cout);
	logging->updateTickCount();

	// create tracker
	tracker = CreateTracker();

#if USE_TEMPLATE_IMAEG
	IplImage* sampleImage = cvLoadImage(TEMPLATE_IMAGE, 0);

	double threahold = tracker->GetDetector()->GetThreshold();
	tracker->GetDetector()->SetThreshold(30.0);
	tracker->AttatchReferenceImage(sampleImage);
	tracker->TrainingReference(SCALE_FACTOR, SCALE_STEP);
	tracker->GetDetector()->SetThreshold(threahold);
#endif

	// initialize rendering engine using GLUT
	renderer = new OpenGLRenderer();
	renderer->Initialize(RENDERING_WIDTH, RENDERING_HEIGHT, "windage Camera Tracjectory");
	renderer->SetCameraSize(WIDTH, HEIGHT);
	
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
