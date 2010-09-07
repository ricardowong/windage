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
#include <vector>

#include <gl/glut.h>
#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/OpenGLRenderer.h"
#include "../Common/FleaCamera.h"

const int NUMBER_OF_REFERENCES = 2;
const char* REFERENCE_IMAGE_FORMAT = "reference%d.png";

const int NUMBER_OF_CAMERAS = 2;
std::vector<FleaCamera*> captures;
std::vector<IplImage*> grayImages;
std::vector<IplImage*> resultImages;
IplImage* resizeImage;
IplImage* colorImage;
IplImage* composeImage;

const double SCALE_FACTOR = 1.0;
const int SCALE_STEP = 1;
const double REPROJECTION_ERROR = 5.0;

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

windage::Logger* logging;
double fps;
const int FPS_UPDATE_STEP = 10;
int fpsStep = 0;

const int FEATURE_COUNT = WIDTH*2;
int keypointCount = 0;
double threshold = 50.0;

std::vector<windage::Frameworks::MultiplePlanarObjectTracking*> trackers;
OpenGLRenderer* renderer = NULL;

bool flip = false;
bool drawCamera = true;

windage::Frameworks::MultiplePlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::MultiplePlanarObjectTracking* tracker = new windage::Frameworks::MultiplePlanarObjectTracking();

	windage::Calibration* calibration = new windage::Calibration();
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SURFdetector();
	windage::Algorithms::SearchTree* searchtree = new windage::Algorithms::FLANNtree();
	windage::Algorithms::OpticalFlow* opticalflow = new windage::Algorithms::OpticalFlow();
	windage::Algorithms::HomographyEstimator* estimator = new windage::Algorithms::RANSACestimator();
	windage::Algorithms::OutlierChecker* checker = new windage::Algorithms::OutlierChecker();
	windage::Algorithms::HomographyRefiner* refiner = new windage::Algorithms::LMmethod();
	windage::Algorithms::KalmanFilter* filter = new windage::Algorithms::KalmanFilter();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(8, 8), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR*3);
	refiner->SetMaxIteration(10);

	tracker->AttatchCalibration(calibration);
	tracker->AttatchDetetor(detector);
//	tracker->AttatchMatcher(searchtree);
	tracker->AttatchTracker(opticalflow);
	tracker->AttatchEstimator(estimator);
	tracker->AttatchChecker(checker);
	tracker->AttatchRefiner(refiner);
//	tracker->AttatchFilter(filter);

	tracker->SetDitectionRatio(30);
	tracker->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	return tracker;
}

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'f':
	case 'F':
		flip = !flip;
		break;
	case 'd':
	case 'D':
		drawCamera = !drawCamera;
		break;
	case 'q':
	case 'Q':

		for(int i=0; i<NUMBER_OF_CAMERAS; i++)
		{
			if(captures[i])
			{
				captures[i]->stop();
				captures[i]->close();
				delete captures[i];
			}
		}
		
		exit(0);
		break;
	}
}

int mode = -1;
int mouseX = -1;
int mouseY = -1;

double ANGLE = 180.0;
double VIRTUAL_CAMERA_DISTANCE = 800.0;

void mouseClick(int button, int state, int x, int y)
{
	if(state == GLUT_DOWN)
	{
		switch(button)
		{
		case GLUT_LEFT_BUTTON:
			mode = 1;
			break;
		case GLUT_RIGHT_BUTTON:
			mode = 2;
			break;
		}
		mouseX = x;
		mouseY = y;
	}
	else if(state == GLUT_UP)
	{
		mode = -1;
	}
}

void mouseMove(int x, int y)
{
	switch(mode)
	{
	case 1:
		ANGLE += ((y - mouseY) + (x - mouseX)) / 100.0;
		break;
	case 2:
		VIRTUAL_CAMERA_DISTANCE += ((y - mouseY) + (x - mouseX)) / 10.0;
		if(VIRTUAL_CAMERA_DISTANCE < 10)
			VIRTUAL_CAMERA_DISTANCE = 10;
		break;
	}
}

void idle(void)
{
	glutPostRedisplay();
}

void display()
{
	// clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw virtual object
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glViewport(0, 0, renderer->width, renderer->height);
	gluPerspective(60, (double)renderer->width/(double)renderer->height, 0.1, 10000.0);

	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	
	double radian = ANGLE * CV_PI / 180.0;
	double dx = sin(radian) * VIRTUAL_CAMERA_DISTANCE;
	double dy = cos(radian) * VIRTUAL_CAMERA_DISTANCE;
	gluLookAt(dx, dy, 2000, 0.0, 0.0, 600.0, 0.0, 0.0, 1.0);

	glPushMatrix();
	{
		windage::Matrix4 translation;
		translation._11 = 1.0;
		translation._22 = 1.0;
		translation._33 = 1.0;
		translation._44 = 1.0;
		translation._42 = 300.0;

		// draw reference image & coordinate
		renderer->DrawReference(0, (double)WIDTH, (double)HEIGHT);
		renderer->DrawAxis((double)WIDTH / 4.0);

//		renderer->DrawReference(1, (double)WIDTH, (double)HEIGHT, translation);

		// capture from camera
		for(int i=0; i<NUMBER_OF_CAMERAS; i++)
		{
			captures[i]->update();
			IplImage* grabImage = captures[i]->GetIPLImage();
			if(flip)
				cvFlip(grabImage, grabImage);
			
			cvResize(grabImage, resizeImage);
			cvCvtColor(resizeImage, colorImage, CV_BGRA2BGR);
			cvCvtColor(resizeImage, grayImages[i], CV_BGRA2GRAY);
			cvCopyImage(colorImage, resultImages[i]);

			// update camera pose
			trackers[i]->UpdateCamerapose(grayImages[i]);

			for(int j=0; j<NUMBER_OF_REFERENCES; j++)
			{
				trackers[i]->DrawDebugInfo(resultImages[i], j);
				trackers[i]->DrawOutLine(resultImages[i], j, true);
				trackers[i]->GetCameraParameter(j)->DrawInfomation(resultImages[i], WIDTH/4);
			}
			int matchingCount = trackers[i]->GetMatchingCount(0);

			// adaptive threshold
			int localcount = trackers[i]->GetDetector()->GetKeypointsCount();
			if(keypointCount != localcount) // if updated
			{
				if(localcount > FEATURE_COUNT)
					threshold += 1;
				if(localcount < FEATURE_COUNT)
					threshold -= 1;
				keypointCount = localcount;
				trackers[i]->GetDetector()->SetThreshold(threshold);
			}

			char message[100];
			sprintf_s(message, "FPS : %.2lf", fps);
			windage::Utils::DrawTextToImage(resultImages[i], cvPoint(10, 20), 0.6, message);
			sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", keypointCount, threshold);
			windage::Utils::DrawTextToImage(resultImages[i], cvPoint(10, 40), 0.6, message);
			sprintf_s(message, "Matching Count : %d", matchingCount);
			windage::Utils::DrawTextToImage(resultImages[i], cvPoint(10, 60), 0.6, message);

			sprintf_s(message, "Press 'Space' to track the current image");
			windage::Utils::DrawTextToImage(resultImages[i], cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
			sprintf_s(message, "Press 'F' to flip image");
			windage::Utils::DrawTextToImage(resultImages[i], cvPoint(WIDTH-270, HEIGHT-25), 0.5, message);

			char windowName[100];
			sprintf(windowName, "tracking %d", i);
			cvShowImage(windowName, resultImages[i]);

			cvSetImageROI(composeImage, cvRect(WIDTH*i, 0, WIDTH*(i+1), HEIGHT));
			cvCopyImage(resultImages[i], composeImage);
		}

		for(int i=0; i<NUMBER_OF_CAMERAS; i++)
		{
			for(int j=1; j<NUMBER_OF_REFERENCES; j++)
			{
				windage::Matrix4 relation = windage::Coordinator::MultiMarkerCoordinator::GetRelation(trackers[i]->GetCameraParameter(0), trackers[i]->GetCameraParameter(j));
				renderer->DrawReference(1, (double)WIDTH, (double)HEIGHT, relation.Transpose(), windage::Vector3(1.0, 0, i));
			}
		}
		if(drawCamera)
		for(int i=0; i<NUMBER_OF_CAMERAS; i++)
		{
			renderer->DrawCamera(i, trackers[i]->GetCameraParameter(0), colorImage);
		}
		
		cvResetImageROI(composeImage);
		renderer->DrawDID(composeImage, 0.2*NUMBER_OF_CAMERAS, 0.2);
	}
	glPopMatrix();

	

	// calculate fps
    fpsStep++;
    if(fpsStep >= FPS_UPDATE_STEP)
    {
		fps = logging->calculateFPS()*(double)FPS_UPDATE_STEP;
		logging->updateTickCount();
		fpsStep = 0;
    }

	glutSwapBuffers();
}

void main()
{
	logging = new windage::Logger(&std::cout);
	logging->updateTickCount();

	captures.resize(NUMBER_OF_CAMERAS);
	grayImages.resize(NUMBER_OF_CAMERAS);
	resultImages.resize(NUMBER_OF_CAMERAS);
	trackers.resize(NUMBER_OF_CAMERAS);

	// connect camera
	resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	colorImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	composeImage = cvCreateImage(cvSize(WIDTH*NUMBER_OF_CAMERAS, HEIGHT), IPL_DEPTH_8U, 3);
	for(int i=0; i<NUMBER_OF_CAMERAS; i++)
	{
		captures[i] = new FleaCamera();
		captures[i]->open();
		captures[i]->start();

		char windowName[100];
		sprintf(windowName, "tracking %d", i);
		cvNamedWindow(windowName);

		grayImages[i] = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
		resultImages[i] = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

		trackers[i] = CreateTracker();

		for(int j=0; j<NUMBER_OF_REFERENCES; j++)
		{
			char referenceName[100];
			sprintf(referenceName, REFERENCE_IMAGE_FORMAT, j+1);

			IplImage* sampleImage = cvLoadImage(referenceName);
			IplImage* grayImage = cvLoadImage(referenceName, 0);

			trackers[i]->AttatchReferenceImage(grayImage);
			trackers[i]->TrainingReference(SCALE_FACTOR, SCALE_STEP);

			cvReleaseImage(&sampleImage);
			cvReleaseImage(&grayImage);
		}
	}

	// initialize rendering engine using GLUT
	renderer = new OpenGLRenderer();
	renderer->Initialize(RENDERING_WIDTH, RENDERING_HEIGHT, "windage Camera Tracjectory");
	renderer->SetCameraSize(WIDTH, HEIGHT);
	renderer->SetNumberOfCameras(NUMBER_OF_CAMERAS);

	for(int j=0; j<NUMBER_OF_REFERENCES; j++)
	{
		char referenceName[100];
		sprintf(referenceName, REFERENCE_IMAGE_FORMAT, j+1);

		IplImage* sampleImage = cvLoadImage(referenceName);

		renderer->AttatchReference(sampleImage);

		cvReleaseImage(&sampleImage);
	}
	
	
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);

	glutMainLoop();

	for(int i=0; i<NUMBER_OF_CAMERAS; i++)
	{
		if(captures[i])
		{
			captures[i]->stop();
			captures[i]->close();
			delete captures[i];
		}
	}	
	cvDestroyAllWindows();
}
