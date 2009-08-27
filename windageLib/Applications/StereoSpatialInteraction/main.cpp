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
#include <windage.h>

#include "PGRCamera.h"
#include "OpenGLRenderer.h"

#define DETECTOR_TYPE StereoSensorDetector
//#define DETECTOR_TYPE StereoSURFDetector
#define UNDISTORTION

const int WIDTH = 640;
const int HEIGHT = 480;
const int SPACE = 10;

const double ACTIVATION_TRESHOLD = 0.2;

IplImage* input1;
IplImage* input2;
IplImage* temp1;
IplImage* temp2;
IplImage* gray1;
IplImage* gray2;
CPGRCamera* camera1;
CPGRCamera* camera2;

windage::Tracker* tracker1;
windage::Tracker* tracker2;

windage::AugmentedReality* arTool;

bool isTracking = true;
SensorGroup* cubeGroup;
//std::vector<windage::SpatialSensor*> spatialSensors;

SensorDetector* sensorDetector;

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'r':
		isTracking = !isTracking;
		break;
	case 'q':
	case 'Q':
		camera1->stop();
		camera1->close();
		camera2->stop();
		camera2->close();
		exit(0);
		break;
	}
}

void idle(void)
{
	// camera frame grabbing
	camera1->update();
	camera2->update();

#ifdef UNDISTORTION
	tracker1->GetCameraParameter()->Undistortion(camera1->GetIPLImage(), temp1);
	tracker2->GetCameraParameter()->Undistortion(camera2->GetIPLImage(), temp2);
#else
	cvCopy(camera1->GetIPLImage(), temp1);
	cvCopy(camera2->GetIPLImage(), temp2);
#endif
	cvResize(temp1, input1);
	cvResize(temp2, input2);

	cvCvtColor(input1, gray1, CV_BGRA2GRAY);
	cvCvtColor(input2, gray2, CV_BGRA2GRAY);

	// call tracking algorithm
	if(isTracking)
	{
		tracker1->UpdateCameraPose(gray1);
		tracker2->UpdateCameraPose(gray2);
	}
	tracker1->DrawInfomation(input1, 100.0);
	tracker2->DrawInfomation(input2, 100.0);

	// calculate spatial sensors stereo-based
	std::vector<IplImage*> images;
	images.push_back(gray1);
	images.push_back(gray2);

	sensorDetector->CalculateActivation(&images);

	cvShowImage("image1", input1);
	cvShowImage("image2", input2);

	glutPostRedisplay();
}

void display()
{
	// draw real scene image
	arTool->DrawBackgroundTexture(input1);

	// apply camera paramter for AR
	arTool->SetProjectionMatrix();
	arTool->SetModelViewMatrix();

	glPushMatrix();
		// draw axis lines
		glLineWidth(5);
		glBegin(GL_LINES);
			glColor3d(1.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(100.0, 0.0, 0.0);
			glColor3d(0.0, 1.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 100.0, 0.0);
			glColor3d(0.0, 0.0, 1.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 0.0, 100.0);
		glEnd();
	glPopMatrix();

	int activeCount = 0;
	// draw spatial sensors

	std::vector<windage::SpatialSensor*>* spatialSensors = cubeGroup->GetSensors();
	for(int i=0; i<spatialSensors->size(); i++)
	{
		glPushMatrix();
			Vector3 position = (*spatialSensors)[i]->GetPosition();
			glTranslated(position.x, position.y, position.z);
			
			glDisable(GL_LIGHTING);
			glEnable(GL_BLEND);

			if((*spatialSensors)[i]->IsActive())
			{
				glColor4f(1, 0, 0, 0.8);
				activeCount++;
			}
			else
			{
				glColor4f(0, 0, 1, 0.2);
			}
			glutSolidCube(((CubeSensorGroup*)cubeGroup)->GetCellSize());

			glEnable(GL_LIGHTING);
			glDisable(GL_BLEND);
		glPopMatrix();
	}

	glutSwapBuffers();
}

void main()
{
	// connect camera
	camera1 = new CPGRCamera();
	camera2 = new CPGRCamera();
	camera1->open();
	camera2->open();
	camera1->start();
	camera2->start();

	input1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	temp1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	temp2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	cvNamedWindow("image1");
	cvNamedWindow("image2");

	// initialize tracker
	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	tracker1 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker1)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker1)->SetOpticalFlowRunning(true);
	tracker1->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);
	tracker2->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	// initialize ar tools
	arTool = new windage::ARForOpenGL();
	((windage::ARForOpenGL*)arTool)->Initialize(WIDTH, HEIGHT, true);
	((windage::ARForOpenGL*)arTool)->AttatchCameraParameter(tracker1->GetCameraParameter());

	// initialize spatial sensors
	cubeGroup = new CubeSensorGroup();
	((CubeSensorGroup*)cubeGroup)->Initialize(1, Vector3(75, 75, 75));

	sensorDetector = new DETECTOR_TYPE();
	((DETECTOR_TYPE*)sensorDetector)->Initialize(ACTIVATION_TRESHOLD, 10.0);
	((DETECTOR_TYPE*)sensorDetector)->AttatchCameraParameter(0, tracker1->GetCameraParameter());
	((DETECTOR_TYPE*)sensorDetector)->AttatchCameraParameter(1, tracker2->GetCameraParameter());

	// attatch sensors
	sensorDetector->AttatchSpatialSensors(cubeGroup->GetSensors());

	// initialize rendering engine
	OpenGLRenderer::init(WIDTH, HEIGHT);
	OpenGLRenderer::setLight();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	
	glutMainLoop();

	camera1->stop();
	camera1->close();
	camera2->stop();
	camera2->close();
}
