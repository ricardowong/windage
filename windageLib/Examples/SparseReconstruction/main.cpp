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
#include <omp.h>

#include <gl/glut.h>
#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/OpenGLRenderer.h"

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;
const double INTRINSIC_VALUES[8] = {WIDTH*0.8, WIDTH*0.8, WIDTH/2, HEIGHT/2, 0, 0, 0, 0};

const int IMAGE_FILE_COUNT = 10;
const char* IMAGE_FILE_NAME = "Reconstruction/test%03d.jpg";
//const char* IMAGE_FILE_NAME = "Test/testImage%d.png";
double VIRTUAL_CAMERA_DISTANCE = 0.5;
const double SCALE_FACTOR = 1.0;
windage::Vector4 centerPoint;

std::vector<IplImage*> inputImage;

OpenGLRenderer* renderer = NULL;
windage::Calibration* initialCalibration;
windage::Reconstruction::IncrementalReconstruction* reconstructor;

int CalculationStep = 2;
windage::Logger* logging;
double threshold = 30.0;
double angle = 0.0;
double angleStep = 1.0;

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
	case 'a':
	case 'A':
		reconstructor->CalculateStep(CalculationStep);
		CalculationStep++;

		{
			// calcuate center Point
			int count = 0;
			centerPoint = windage::Vector4(0.0, 0.0, 0.0, 0.0);
			std::vector<windage::ReconstructionPoint>* point3D = reconstructor->GetReconstructedPoint();
			for(unsigned int j=0; j<point3D->size(); j++)
			{
				centerPoint += (*point3D)[j].GetPoint();
				count++;
			}
			centerPoint /= (double)count;
			VIRTUAL_CAMERA_DISTANCE = centerPoint.getLength();
			logging->log("reconstruction point count : ");logging->log(count); logging->logNewLine();
		}
		break;
	case 'q':
	case 'Q':
		cvDestroyAllWindows();
		exit(0);
		break;
	}
}

void idle(void)
{
	angle += angleStep;
	if(angle >= 360.0)
		angle = 0.0;
	glutPostRedisplay();
}

void display()
{
	// clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw virtual object
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	double radian = angle * CV_PI / 180.0;
	double dx = sin(radian) * VIRTUAL_CAMERA_DISTANCE;
	double dz = cos(radian) * VIRTUAL_CAMERA_DISTANCE;
	gluLookAt(centerPoint.x+dx, centerPoint.y-VIRTUAL_CAMERA_DISTANCE/2.0, centerPoint.z+dz, centerPoint.x, centerPoint.y, centerPoint.z, 0.0, -1.0, 0.0);

	glPushMatrix();
	{
		std::vector<windage::ReconstructionPoint>* point3D = reconstructor->GetReconstructedPoint();

		for(unsigned int j=0; j<point3D->size(); j++)
		{
			if((*point3D)[j].IsOutlier() == false)
			{
				int size = (int)(*point3D)[j].GetFeatureList()->size();
				glPointSize(size * 2);

				glBegin(GL_POINTS);
				{
					CvScalar color = (*point3D)[j].GetColor();
					windage::Vector4 point = (*point3D)[j].GetPoint();
					glColor3f(color.val[2]/255.0, color.val[1]/255.0, color.val[0]/255.0);
					glVertex3f(point.x, point.y, point.z);
				}
				glEnd();
			}
		}

		for(int i=0; i<IMAGE_FILE_COUNT; i++)
		{
			renderer->DrawCamera(reconstructor->GetCameraParameter(i), inputImage[i], 0.0005);
//			renderer->DrawCameraAxis(reconstructor->GetCameraParameter(i));
		}

		renderer->DrawAxis((double)RENDERING_WIDTH / 4.0);

	}
	glPopMatrix();

	glutSwapBuffers();
}

void main()
{
	std::vector<IplImage*> grayImage;
	inputImage.resize(IMAGE_FILE_COUNT);
	grayImage.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> featurePoint;
	featurePoint.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> matchedPoint;
	matchedPoint.resize((IMAGE_FILE_COUNT-1)*2);

	logging = new windage::Logger(&std::cout);
	logging->log("initialize"); logging->logNewLine();

	initialCalibration = new windage::Calibration();
	initialCalibration->Initialize(INTRINSIC_VALUES[0], INTRINSIC_VALUES[1], INTRINSIC_VALUES[2], INTRINSIC_VALUES[3]);

	reconstructor = new windage::Reconstruction::IncrementalReconstruction();
	reconstructor->SetConfidence(0.995);
	reconstructor->SetMaxIteration(2000);
	reconstructor->SetReprojectionError(2.0);

	reconstructor->AttatchCalibration(initialCalibration);
	reconstructor->AttatchSearchTree(new windage::Algorithms::FLANNtree());

	windage::Algorithms::OpenCVRANSACestimator* estimator = new windage::Algorithms::OpenCVRANSACestimator();
	estimator->SetMaxIteration(2000);
	reconstructor->AttatchEstimator(estimator);

	logging->logNewLine();
	logging->log("load image & feature extract - matching"); logging->logNewLine();
	
	char filename[100];
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		sprintf_s(filename, IMAGE_FILE_NAME, i);
		inputImage[i] = cvLoadImage(filename);
		grayImage[i] = cvCreateImage(cvGetSize(inputImage[i]), IPL_DEPTH_8U, 1);

		cvCvtColor(inputImage[i], grayImage[i], CV_BGR2GRAY);

		logging->log("\tload image "); logging->log(i); logging->log(" : "); logging->log(filename); logging->logNewLine();
	}
	
	logging->logNewLine();
	logging->log("\tfeature extract"); logging->logNewLine();

	#pragma omp parallel for
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTdetector();
		detector->DoExtractKeypointsDescriptor(grayImage[i]);
		std::vector<windage::FeaturePoint>* temp = detector->GetKeypoints();

		featurePoint[i].clear();
		for(unsigned int j=0; j<temp->size(); j++)
		{
			(*temp)[j].SetColor(cvGet2D(inputImage[i], (*temp)[j].GetPoint().y, (*temp)[j].GetPoint().x));
			featurePoint[i].push_back((*temp)[j]);
		}

		delete detector;

		logging->log("\tkeypoint count "); logging->log(i); logging->log(" : "); logging->log((int)featurePoint[i].size()); logging->logNewLine();
	}

	logging->logNewLine();
	logging->log("reconstruction sequence"); logging->logNewLine();
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		reconstructor->AttatchFeaturePoint(&featurePoint[i]);
	}
//	reconstructor->CalculateAll();
	reconstructor->CalculateStep(CalculationStep);
	CalculationStep++;


	// calcuate center Point
	int count = 0;
	centerPoint = windage::Vector4(0.0, 0.0, 0.0, 0.0);
	std::vector<windage::ReconstructionPoint>* point3D = reconstructor->GetReconstructedPoint();
	for(unsigned int j=0; j<point3D->size(); j++)
	{
		centerPoint += (*point3D)[j].GetPoint();
		count++;
	}
	centerPoint /= (double)count;
	VIRTUAL_CAMERA_DISTANCE = centerPoint.getLength();
	logging->log("reconstruction point count : ");logging->log(count); logging->logNewLine();

	logging->logNewLine();
	logging->log("draw result"); logging->logNewLine();

	// initialize rendering engine using GLUT
	renderer = new OpenGLRenderer();
	renderer->Initialize(RENDERING_WIDTH, RENDERING_HEIGHT, "Sparse Reconstruction");
	
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();

	cvDestroyAllWindows();
}
