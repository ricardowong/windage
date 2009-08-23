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

#include "PGRCamera.h"

#include "OpenGLRenderer.h"
#include "AugmentedReality/ARForOpenGL.h"
#include "Tracker/ModifiedSURFTracker.h"

const int WIDTH = 640;
const int HEIGHT = 480;

CPGRCamera* camera;
windage::Tracker* tracker;
windage::AugmentedReality* arTool;
IplImage* input;
IplImage* gray;

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'q':
	case 'Q':
		camera->stop();
		camera->close();
		exit(0);
		break;
	}
}

void idle(void)
{
	// camera frame grabbing
	camera->update();
	cvResize(camera->GetIPLImage(), input);
	cvCvtColor(input, gray, CV_BGRA2GRAY);

	// call tracking algorithm
	tracker->UpdateCameraPose(gray);

	glutPostRedisplay();
}

void display()
{
	// draw real scene image
	arTool->DrawBackgroundTexture(input);

	// apply camera paramter for AR
	arTool->SetProjectionMatrix();
	arTool->SetModelViewMatrix();

	glPushMatrix();
		// axis lines
		glLineWidth(5);
		glBegin(GL_LINES);
			glColor3d(1.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(10.0, 0.0, 0.0);
			glColor3d(0.0, 1.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 10.0, 0.0);
			glColor3d(0.0, 0.0, 1.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 0.0, 10.0);
		glEnd();
		glTranslated(10, 10, 5);

		// draw virtual object
		OpenGLRenderer::setMaterial(Vector4(0, 0, 255, 0.5));
		glutSolidCube(10);
	glPopMatrix();

	glutSwapBuffers();
}

void main()
{
	// connect camera
	camera = new CPGRCamera();
	camera->open();
	camera->start();

	input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// initialize tracker
	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	tracker = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker)->SetOpticalFlowRunning(true);

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

	camera->stop();
	camera->close();
}