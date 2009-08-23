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

#include "PGRCamera.h"

#include "OpenGLRenderer.h"
#include "AugmentedReality/ARForOpenGL.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "SpatialInteraction/StereoSpatialSensor.h"
#include "SpatialInteraction/StereoSURFSpatialSensor.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const int SPACE = 10;

const double ACTIVATION_TRESHOLD = 0.2;

IplImage* input1;
IplImage* input2;
IplImage* gray1;
IplImage* gray2;
CPGRCamera* camera1;
CPGRCamera* camera2;

windage::Tracker* tracker1;
windage::Tracker* tracker2;

windage::AugmentedReality* arTool;

bool isTracking = true;
std::vector<SpatialSensor*> spatialSensors;

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
	cvResize(camera1->GetIPLImage(), input1);
	cvResize(camera2->GetIPLImage(), input2);
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

	#pragma omp parallel for
	for(int i=0; i<spatialSensors.size(); i++)
	{		
		spatialSensors[i]->CalculateActivation(&images);
	}

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

	for(int i=0; i<spatialSensors.size(); i++)
	{
		glPushMatrix();
			glTranslated(spatialSensors[i]->GetPosition().x, spatialSensors[i]->GetPosition().y, spatialSensors[i]->GetPosition().z);
			
			glDisable(GL_LIGHTING);
			glEnable(GL_BLEND);

			if(spatialSensors[i]->IsActive())
			{
				glColor4f(1, 0, 0, 0.8);
				activeCount++;
			}
			else
			{
				glColor4f(0, 0, 1, 0.2);
			}
			glutSolidCube(SPACE);

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
	gray1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
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

	tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);

	// initialize ar tools
	arTool = new windage::ARForOpenGL();
	((windage::ARForOpenGL*)arTool)->Initialize(WIDTH, HEIGHT, true);
	((windage::ARForOpenGL*)arTool)->AttatchCameraParameter(tracker1->GetCameraParameter());

	// initialize spatial sensors
	const int count = 5;
	for(int z=5; z<5+count; z++)
	{
		for(int y=1; y<count; y++)
		{
			for(int x=1; x<count; x++)
			{
				StereoSURFSpatialSensor* tempSpatialSensor = new StereoSURFSpatialSensor();
				tempSpatialSensor->Initialize(Vector3(x*SPACE*2, y*SPACE*2, z*SPACE*2), ACTIVATION_TRESHOLD);
				tempSpatialSensor->AttatchCameraParameter(0, tracker1->GetCameraParameter());
				tempSpatialSensor->AttatchCameraParameter(1, tracker2->GetCameraParameter());

				spatialSensors.push_back(tempSpatialSensor);
			}
		}
	}

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
