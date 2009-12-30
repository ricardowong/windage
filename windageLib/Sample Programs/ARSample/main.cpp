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

#include "AugmentedReality/ARForOpenGL.h"
#include "OpenGLRenderer.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Logger* logging;
double fps = 0;
const int FPS_UPDATE_STEP = 30;
int fpsStep = 0;

// adaptive threshold
#define ADAPTIVE_THRESHOLD
int fastThreshold = 70;
const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 40;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

CvCapture* capture;
windage::ModifiedSURFTracker* tracker;
windage::AugmentedReality* arTool;
IplImage* input;
IplImage* gray;

using namespace windage;

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, 267.0, 200.0, 4.0, 8);
	tracker->SetPoseEstimationMethod(windage::RANSAC);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'q':
	case 'Q':
		cvReleaseCapture(&capture);
		exit(0);
		break;
	}
}

void idle(void)
{
	glutPostRedisplay();
}

void display()
{
	// camera frame grabbing
	IplImage* grabFrame = cvQueryFrame(capture);
	tracker->GetCameraParameter()->Undistortion(grabFrame, input);
//	cvFlip(input, input);
	cvCvtColor(input, gray, CV_BGRA2GRAY);

	// call tracking algorithm
	tracker->SetFeatureExtractThreshold(fastThreshold);
	tracker->UpdateCameraPose(gray);
//	tracker->DrawInfomation(input);
	tracker->DrawDebugInfo(input);

#ifdef ADAPTIVE_THRESHOLD
	int featureCount = tracker->GetFeatureCount();
	if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
	else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

	// calculate fps
	fpsStep++;
	if(fpsStep > FPS_UPDATE_STEP)
	{
		fps = logging->calculateFPS()*(double)FPS_UPDATE_STEP;
		logging->updateTickCount();
		fpsStep = 0;
	}
	char message[100];
	sprintf(message, "FPS : %.2lf", fps);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 40), message);

	// draw real scene image
	arTool->DrawBackgroundTexture(input);

	// apply camera paramter for AR
	arTool->SetProjectionMatrix();
	arTool->SetModelViewMatrix();

	glPushMatrix();
		// draw virtual object
		OpenGLRenderer::setMaterial(Vector4(255, 255, 255, 0.8));
		glTranslated(0, 0, 5);
		glRotatef(90, 1, 0, 0);

		glutWireTeapot(50);
//		glutSolidTeapot(5);
//		glutSolidCube(10);
	glPopMatrix();

	glutSwapBuffers();
	glutPostRedisplay();
}

void main()
{
	// connect camera
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	gray = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// initialize tracker
	IplImage* referenceImage = cvLoadImage("reference1_320.png", 0);
	tracker = CreateTracker(referenceImage, 0);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	logging = new windage::Logger(&std::cout);
	logging->updateTickCount();

	// initialize ar tools
	arTool = new windage::ARForOpenGL();
	((windage::ARForOpenGL*)arTool)->Initialize(WIDTH, HEIGHT, true);
	((windage::ARForOpenGL*)arTool)->AttatchCameraParameter(tracker->GetCameraParameter());

	// initialize rendering engine using GLUT
	OpenGLRenderer::init(WIDTH, HEIGHT);
	OpenGLRenderer::setLight();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();

	cvReleaseCapture(&capture);
}