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
	camera->update();
	cvResize(camera->GetIPLImage(), input);
	cvCvtColor(input, gray, CV_BGRA2GRAY);

	tracker->UpdateCameraPose(gray);

	glutPostRedisplay();
}

void display()
{
	arTool->DrawBackgroundTexture(input);
	arTool->SetProjectionMatrix();
	arTool->SetModelViewMatrix();

	glPushMatrix();
	// axis lines
//*
		glLineWidth(5);
		glBegin(GL_LINES);
			glColor3d(1.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(10.0, 0.0, 0.0);
			glColor3d(0.0, 1.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 10.0, 0.0);
			glColor3d(0.0, 0.0, 1.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 0.0, 10.0);
		glEnd();
//*/
		glTranslated(10, 10, 5);
		OpenGLRenderer::setMaterial(Vector4(0, 0, 255, 0.8));
		glutSolidCube(10);

	glPopMatrix();
	glutSwapBuffers();
}

void main()
{
	camera = new CPGRCamera();
	camera->open();
	camera->start();

	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	tracker = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 45);
	((windage::ModifiedSURFTracker*)tracker)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker)->SetOpticalFlowRunning(true);

	arTool = new windage::ARForOpenGL();
	((windage::ARForOpenGL*)arTool)->Initialize(WIDTH, HEIGHT, true);
	((windage::ARForOpenGL*)arTool)->AttatchCameraParameter(tracker->GetCameraParameter());

	input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// initialize rendering engine
	OpenGLRenderer::init(WIDTH, HEIGHT);
	OpenGLRenderer::setLight();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();

	camera->stop();
	camera->close();
}