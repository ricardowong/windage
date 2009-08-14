#include <iostream>

#include "PGRCamera.h"

#include "OpenGLRenderer.h"
#include "AugmentedReality/ARForOpenGL.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "Tracker/ChessboardTracker.h"

#include "ball.h"

const int WIDTH = 320;
const int HEIGHT = 240;
const float SPACE = 100; 

Ball ball;

CPGRCamera* camera1;
CPGRCamera* camera2;
windage::Tracker* tracker1;
windage::Tracker* tracker2;
windage::AugmentedReality* arTool;
IplImage* input1;
IplImage* input2;
IplImage* gray1;
IplImage* gray2;

const int WINDOW_SIZE = 15;
IplImage* kernel1;
IplImage* kernel2;
IplImage* disparity;

bool isTracking = true;

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'r':
		isTracking = !isTracking;
		break;
	case '[':
		ball.SpeedDown();
		ch = 0;
		break;
	case ']':
		ball.SpeedUp();
		ch = 0;
		break;
	case ' ':
		ball.Reset(SPACE);
		ch = 0;
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

void GetKernel(IplImage* image, IplImage* kernel, CvPoint point, int windowSize=WINDOW_SIZE/2)
{
	cvZero(kernel);
	for(int y=-windowSize; y<windowSize; y++)
	{
		for(int x=-windowSize; x<windowSize; x++)
		{
			if(0 <= point.x + x && point.x + x < image->width &&
				0 <= point.y + y && point.y + y < image->height)
			{
				cvSet2D(kernel, y+windowSize, x+windowSize, cvGet2D(image, point.y + y, point.x + x));
			}
		}
	}
}
double CalcDisparity(IplImage* disparity, IplImage* kernel1, IplImage* kernel2)
{
	cvZero(disparity);
	double distance = 0;
	for(int y=0; y<disparity->height; y++)
	{
		for(int x=0; x<disparity->width; x++)
		{
			cvSetReal2D(disparity, y, x, abs(cvGetReal2D(kernel1, y, x) - cvGetReal2D(kernel2, y, x)));
			distance += abs(cvGetReal2D(kernel1, y, x) - cvGetReal2D(kernel2, y, x));
		}
	}
	return distance;
}

void idle(void)
{
	camera1->update();
	camera2->update();
	cvResize(camera1->GetIPLImage(), input1);
	cvResize(camera2->GetIPLImage(), input2);
	cvCvtColor(input1, gray1, CV_BGRA2GRAY);
	cvCvtColor(input2, gray2, CV_BGRA2GRAY);

	if(isTracking)
	{
		tracker1->UpdateCameraPose(gray1);
		tracker2->UpdateCameraPose(gray2);
		tracker2->DrawDebugInfo(input2);
	}
	tracker2->DrawInfomation(input2, 100.0);

	Vector3 ballCoordinate = ball.CalcWorldCoordinate();

	CvPoint point;
	point = tracker1->GetCameraParameter()->ConvertWorld2Image(ballCoordinate.x, ballCoordinate.y, ballCoordinate.z);
	GetKernel(gray1, kernel1, point);
	cvRectangle(input1, cvPoint(point.x - WINDOW_SIZE/2, point.y - WINDOW_SIZE/2), cvPoint(point.x + WINDOW_SIZE/2, point.y + WINDOW_SIZE/2), CV_RGB(255, 0, 0));

	point = tracker2->GetCameraParameter()->ConvertWorld2Image(ballCoordinate.x, ballCoordinate.y, ballCoordinate.z);
	GetKernel(gray2, kernel2, point);
	cvRectangle(input2, cvPoint(point.x - WINDOW_SIZE/2, point.y - WINDOW_SIZE/2), cvPoint(point.x + WINDOW_SIZE/2, point.y + WINDOW_SIZE/2), CV_RGB(255, 0, 0));

	double distance = CalcDisparity(disparity, kernel1, kernel2);
	distance /= (double)(WINDOW_SIZE * WINDOW_SIZE);
	std::cout << distance << std::endl;

	if(distance < WINDOW_SIZE/2)
		ball.direction = -ball.direction;

	ball.CollisoinDetect();
	ball.UpdatePosition();

	cvShowImage("image1", input1);
	cvShowImage("image2", input2);
	cvShowImage("kernel1", kernel1);
	cvShowImage("kernel2", kernel2);
	cvShowImage("disparity", disparity);

	glutPostRedisplay();
}

void display()
{
	// for Augmented Reality
	arTool->DrawBackgroundTexture(input1);
	arTool->SetProjectionMatrix();
	arTool->SetModelViewMatrix();

	glPushMatrix();
	// draw axis lines
//*
		glLineWidth(5);
		glBegin(GL_LINES);
			glColor3d(1.0, 0.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(100.0, 0.0, 0.0);
			glColor3d(0.0, 1.0, 0.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 100.0, 0.0);
			glColor3d(0.0, 0.0, 1.0);
			glVertex3d(0.0, 0.0, 0.0);glVertex3d(0.0, 0.0, 100.0);
		glEnd();
//*/
	glPopMatrix();

	// draw ball
	ball.SetBasePosition();
	glPushMatrix();
		glTranslated(ball.GetPosition().x, ball.GetPosition().y, ball.GetPosition().z);
		ball.Draw();
	glPopMatrix();

	// boundary space
	glPushMatrix();

		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);

		glColor4f(1, 1, 1, 1);
		glBegin(GL_LINES);
			glVertex3f(-SPACE, -SPACE, -SPACE); glVertex3f(+SPACE, -SPACE, -SPACE);
			glVertex3f(-SPACE, +SPACE, -SPACE); glVertex3f(+SPACE, +SPACE, -SPACE);
			glVertex3f(-SPACE, -SPACE, +SPACE);	glVertex3f(+SPACE, -SPACE, +SPACE);
			glVertex3f(-SPACE, +SPACE, +SPACE);	glVertex3f(+SPACE, +SPACE, +SPACE);

			glVertex3f(-SPACE, -SPACE, -SPACE); glVertex3f(-SPACE, +SPACE, -SPACE);
			glVertex3f(+SPACE, -SPACE, -SPACE); glVertex3f(+SPACE, +SPACE, -SPACE);
			glVertex3f(-SPACE, -SPACE, +SPACE);	glVertex3f(-SPACE, +SPACE, +SPACE);
			glVertex3f(+SPACE, -SPACE, +SPACE);	glVertex3f(+SPACE, +SPACE, +SPACE);

			glVertex3f(-SPACE, -SPACE, -SPACE); glVertex3f(-SPACE, -SPACE, +SPACE);
			glVertex3f(-SPACE, +SPACE, -SPACE); glVertex3f(-SPACE, +SPACE, +SPACE);
			glVertex3f(+SPACE, -SPACE, -SPACE);	glVertex3f(+SPACE, -SPACE, +SPACE);
			glVertex3f(+SPACE, +SPACE, -SPACE); glVertex3f(+SPACE, +SPACE, +SPACE);
		glEnd();

		glColor4f(0, 0, 1, 0.5);
		glutSolidCube(SPACE * 2);

		glEnable(GL_LIGHTING);
		glDisable(GL_BLEND);

	glPopMatrix();

	glutSwapBuffers();
}

void main()
{
	camera1 = new CPGRCamera();
	camera2 = new CPGRCamera();
	camera1->open();
	camera2->open();
	camera1->start();
	camera2->start();

//*
	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	tracker1 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 20);
//	((windage::ModifiedSURFTracker*)tracker1)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 45);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker1)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker1)->SetOpticalFlowRunning(true);
	tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(535.703, 539.716, 158.839, 98.400, -0.277075, 0.938586, -0.010295, -0.006803, 20);
//	((windage::ModifiedSURFTracker*)tracker2)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);
//*/

/*
	tracker1 = new windage::ChessboardTracker();
	((windage::ChessboardTracker*)tracker1)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 28.0);
	tracker2 = new windage::ChessboardTracker();
	((windage::ChessboardTracker*)tracker2)->Initialize(1071.406, 1079.432, 317.678, 196.800, -0.277075, 0.938586, -0.010295, -0.006803, 7, 8, 28.0);
//*/
	arTool = new windage::ARForOpenGL();
	((windage::ARForOpenGL*)arTool)->Initialize(WIDTH, HEIGHT, true);
	((windage::ARForOpenGL*)arTool)->AttatchCameraParameter(tracker1->GetCameraParameter());

	input1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	gray2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	kernel1 = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 1);
	kernel2 = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 1);
	disparity = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 1);

	ball.Reset(SPACE);

	// initialize rendering engine
	OpenGLRenderer::init(WIDTH, HEIGHT);
	OpenGLRenderer::setLight();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	cvNamedWindow("image1");
	cvNamedWindow("image2");
	cvNamedWindow("kernel1");
	cvNamedWindow("kernel2");
	cvNamedWindow("disparity");

	glutMainLoop();

	camera1->stop();
	camera1->close();
	camera2->stop();
	camera2->close();
}