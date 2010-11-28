/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
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

#include "../Common/OpenGLRendererEx.h"

#include <windage.h>
#include <Coordinator/ARForOpenGL.h>

const char* FILE_NAME = "testImages\\reference6.png";
const char* RESULT_DIR = "testImages\\reference6_d-%d_%c-%d.png";
const int WIDTH = 640;
const int HEIGHT = 480;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

const int ROTATION_RANGE_S = 0;
const int ROTATION_RANGE_E = +80;
const int ROTATION_RANGE_D = 10;

const int DISTANCE_RANGE_S = -600;
const int DISTANCE_RANGE_E = 2000;
const int DISTANCE_RANGE_D = 300;

windage::Coordinator::AugmentedReality* artool = NULL;
OpenGLRenderer* renderer = NULL;

IplImage* image;
IplImage* saveImage;

void keyboard(unsigned char ch, int x, int y)
{
	char message[1000];
	switch(ch)
	{
	case 's':
	case 'S':
		break;
	case 'q':
	case 'Q':
	case 27:
		exit(0);
		break;
	default:
		glutPostRedisplay();
		break;
	}
}

void idle(void)
{
//	glutPostRedisplay();
}

void display()
{
	char message[1000];
	for(int d=DISTANCE_RANGE_S; d<=DISTANCE_RANGE_E; d+=DISTANCE_RANGE_D)
	{
		for(int rMode=0; rMode<3; rMode++)
		{
			for(int r=ROTATION_RANGE_S; r<=ROTATION_RANGE_E; r+=ROTATION_RANGE_D)
			{
				// clear screen
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				// draw virtual object
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();

				// apply camera paramter for AR
				artool->SetProjectionMatrix();
			//	artool->SetModelViewMatrix();

				double distance = -INTRINSIC[0];
				double error = 10.0;
				gluLookAt(0.0, 0.0, distance, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);

				// transformation
				glTranslatef(0.0f, 0.0f, (float)d);
				switch(rMode)
				{
				case 0://x
					glRotatef((float)r, 1.0f, 0.0f, 0.0f);
					break;
				case 1://y
					glRotatef((float)r, 0.0f, 1.0f, 0.0f);
					break;
				case 2://z
					glRotatef((float)r, 0.0f, 0.0f, 1.0f);
					break;
				}

				// draw real scene image
			//	artool->DrawBackgroundTexture(image);

				renderer->AttatchReference(image);
				renderer->DrawReference((double)WIDTH, (double)HEIGHT);
			//	renderer->DrawAxis((double)WIDTH / 4.0);

				glutSwapBuffers();
				glutSwapBuffers();
//				glutPostRedisplay();

				Sleep(50);

				// save image
				glPixelStorei(GL_PACK_ALIGNMENT, 0);
				glReadPixels(0, 0, WIDTH, HEIGHT, GL_BGR_EXT, GL_UNSIGNED_BYTE, saveImage->imageData);

				int tempR = (r<0)?360+r:r;
				int tempD = d - DISTANCE_RANGE_S;
				switch(rMode)
				{
				case 0://x
					sprintf(message, RESULT_DIR, tempD, 'x', tempR);
					break;
				case 1://y
					sprintf(message, RESULT_DIR, tempD, 'y', tempR);
					break;
				case 2://z
					sprintf(message, RESULT_DIR, tempD, 'z', tempR);
					break;
				}
				cvSaveImage(message, saveImage);
				std::cout << message << std::endl;

				Sleep(50);

				glutSwapBuffers();
			}
		}
	}
}

void main()
{
	image = cvLoadImage(FILE_NAME);
	saveImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	saveImage->origin = 1;

	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	// create and initialize AR tool
	artool = new windage::Coordinator::ARForOpenGL();
	((windage::Coordinator::ARForOpenGL*)artool)->Initialize(WIDTH, HEIGHT, true);
	artool->AttatchCameraParameter(calibration);

	renderer = new OpenGLRenderer();
	renderer->Initialize(WIDTH, HEIGHT, "windageLib Simple AR");
	renderer->SetCameraSize(WIDTH, HEIGHT);

	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	glutMainLoop();
}