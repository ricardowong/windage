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

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;
const double INTRINSIC_VALUES[8] = {WIDTH*1.2, WIDTH*1.2, WIDTH/2, HEIGHT/2, 0, 0, 0, 0};

const int IMAGE_FILE_COUNT = 2;
const char* IMAGE_FILE_NAME = "test%02d.jpg";
const double VIRTUAL_CAMERA_DISTANCE = 800.0;

OpenGLRenderer* renderer = NULL;
windage::Calibration* calibration1 = NULL;
windage::Calibration* calibration2 = NULL;
windage::Algorithms::FeatureDetector* detector = NULL;
windage::Algorithms::SearchTree* searchtree = NULL;
windage::Reconstruction::StereoReconstruction* stereo = NULL;

windage::Logger* logging;
double threshold = 30.0;
double angle = 0.0;
double angleStep = 0.5;

void keyboard(unsigned char ch, int x, int y)
{
	switch(ch)
	{
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
	double dy = cos(radian) * VIRTUAL_CAMERA_DISTANCE;
	gluLookAt(dx, dy, 800, 0.0, 0.0, 300.0, 0.0, 0.0, 1.0);

	glPushMatrix();
	{
		std::vector<windage::ReconstructionPoint>* point3D = stereo->GetReconstructionPoints();

		glPointSize(5.0f);
		glBegin(GL_POINTS);
		{
			for(unsigned int i=0; i<point3D->size(); i++)
			{
				if((*point3D)[i].IsOutlier() == false)
				{
					CvScalar color = (*point3D)[i].GetColor();
					windage::Vector4 point = (*point3D)[i].GetPoint();
					glColor3f(color.val[2]/255.0, color.val[1]/255.0, color.val[0]/255.0);
					glVertex3f(point.x, point.y, point.z);
				}
			}
		}
		glEnd();
		
		renderer->DrawAxis((double)RENDERING_WIDTH / 4.0);
	}
	glPopMatrix();

	glutSwapBuffers();
}

void main()
{
	std::vector<IplImage*> inputImage;
	std::vector<IplImage*> grayImage;
	inputImage.resize(IMAGE_FILE_COUNT);
	grayImage.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> featurePoint;
	featurePoint.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> matchedPoint;
	matchedPoint.resize(IMAGE_FILE_COUNT);

	logging = new windage::Logger(&std::cout);
	logging->log("initialize"); logging->logNewLine();
	
	calibration1 = new windage::Calibration();
	calibration2 = new windage::Calibration();
	calibration1->Initialize(INTRINSIC_VALUES[0], INTRINSIC_VALUES[1], INTRINSIC_VALUES[2], INTRINSIC_VALUES[3]);
	calibration2->Initialize(INTRINSIC_VALUES[0], INTRINSIC_VALUES[1], INTRINSIC_VALUES[2], INTRINSIC_VALUES[3]);
	
	stereo = new windage::Reconstruction::StereoReconstruction();
	stereo->AttatchBaseCameraParameter(calibration1);
	stereo->AttatchUpdateCameraParameter(calibration2);

	detector = new windage::Algorithms::SIFTdetector();
	searchtree = new windage::Algorithms::FLANNtree();

	logging->logNewLine();
	logging->log("load image & feature extract - matching"); logging->logNewLine();
	
	char filename[100];
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		sprintf_s(filename, IMAGE_FILE_NAME, i);
		inputImage[i] = cvLoadImage(filename);
		grayImage[i] = cvCreateImage(cvGetSize(inputImage[i]), IPL_DEPTH_8U, 1);

		cvCvtColor(inputImage[i], grayImage[i], CV_BGR2GRAY);

		logging->log("\tload image : "); logging->log(filename); logging->logNewLine();
	}
	
	logging->logNewLine();
	logging->log("\tfeature extract"); logging->logNewLine();

	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		detector->DoExtractKeypointsDescriptor(grayImage[i]);
		std::vector<windage::FeaturePoint>* temp = detector->GetKeypoints();
		for(unsigned int j=0; j<temp->size(); j++)
			featurePoint[i].push_back((*temp)[j]);

		logging->log("\tkeypoint count : "); logging->log((int)featurePoint[i].size()); logging->logNewLine();
	}

	logging->logNewLine();
	logging->log("\tfeature matching"); logging->logNewLine();

	searchtree->Training(&featurePoint[0]);
	for(unsigned int i=0; i<featurePoint[1].size(); i++)
	{
		int index = searchtree->Matching(featurePoint[1][i]);
		if(index >= 0)
		{
			matchedPoint[0].push_back(featurePoint[0][index]);
			matchedPoint[1].push_back(featurePoint[1][i]);
		}
	}
	int matchCount = (int)matchedPoint[0].size();
	logging->log("\tmatching count : "); logging->log(matchCount); logging->logNewLine();

	logging->logNewLine();
	logging->log("reconstruction"); logging->logNewLine();

	stereo->AttatchMatchedPoint1(&matchedPoint[0]);
	stereo->AttatchMatchedPoint2(&matchedPoint[1]);

	double error = 0.0;
	stereo->CalculateNormalizedPoint();
	stereo->ComputeEssentialMatrixRANSAC(&error);
	int inlierCount = stereo->GetInlierCount();
	logging->log("\tinlier ratio : "); logging->log(inlierCount); logging->log("/"); logging->log(matchCount); logging->logNewLine();
	logging->log("\terror : "); logging->log(error); logging->logNewLine();

	logging->logNewLine();
	logging->log("color matching"); logging->logNewLine();

	std::vector<windage::ReconstructionPoint>* point3D = stereo->GetReconstructionPoints();
	for(unsigned int i=0; i<point3D->size(); i++)
	{
		if((*point3D)[i].IsOutlier() == false)
		{
			CvScalar color1 = cvGet2D(inputImage[0], matchedPoint[0][i].GetPoint().y, matchedPoint[0][i].GetPoint().x);
			CvScalar color2 = cvGet2D(inputImage[1], matchedPoint[1][i].GetPoint().y, matchedPoint[1][i].GetPoint().x);

			(*point3D)[i].SetColor(CV_RGB(	(color1.val[2]+color2.val[2])/2,
											(color1.val[1]+color2.val[1])/2,
											(color1.val[0]+color2.val[0])/2 ) );
		}
	}

	logging->logNewLine();
	logging->log("scale up"); logging->logNewLine();

	for(unsigned int i=0; i<point3D->size(); i++)
	{
		(*point3D)[i].SetPoint((*point3D)[i].GetPoint() * 80.0);
	}

	logging->logNewLine();
	logging->log("camera pose information"); logging->logNewLine();
	logging->log("\tintrinsic"); logging->logNewLine();
	logging->log(calibration1->GetIntrinsicMatrix());
	logging->log("\tcamera1"); logging->logNewLine();
	logging->log(calibration1->GetExtrinsicMatrix());
	logging->log("\tcamera2"); logging->logNewLine();
	logging->log(calibration2->GetExtrinsicMatrix());


	logging->logNewLine();
	logging->log("draw result"); logging->logNewLine();


	logging->logNewLine();
	logging->log("release memory"); logging->logNewLine();
	delete detector;	detector = NULL;
	delete searchtree;	searchtree = NULL;

	// initialize rendering engine using GLUT
	renderer = new OpenGLRenderer();
	renderer->Initialize(RENDERING_WIDTH, RENDERING_HEIGHT, "Sparse Reconstruction");
	
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();

	cvDestroyAllWindows();
}
