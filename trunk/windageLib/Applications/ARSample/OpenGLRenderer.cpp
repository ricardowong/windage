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

#include "OpenGLRenderer.h"

#include <stdio.h>

#define BUFSIZE 64
//#define DRAWPIXEL_MODE

OpenGLRenderer::MODE OpenGLRenderer::mode = OpenGLRenderer::MODE::RENDER;

int OpenGLRenderer::cursorX = 0;
int OpenGLRenderer::cursorY = 0;

int OpenGLRenderer::windowWidth = 0;
int OpenGLRenderer::windowHeight = 0;

int OpenGLRenderer::cameraWidth = 320;
int OpenGLRenderer::cameraHeight = 240;

void OpenGLRenderer::init(int width, int height)
{
	cameraWidth = windowWidth = width;
	cameraHeight = windowHeight = height;

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutCreateWindow("OpenGL Renderer");
	glutReshapeWindow(width, height);
//	glutDisplayFunc(display);
//	glutIdleFunc(idle);

//*
	glEnable(GL_DEPTH_TEST);
//	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_ALPHA_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	glEnable(GL_BLEND);
//*/
	glClearColor(1.0, 1.0, 1.0, 1.0);
}

void OpenGLRenderer::drawClear()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRenderer::setLight()
{
	GLfloat diffuse0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat ambient0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat specular0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat light0_pos[]={0.0, 250.0, 250, 1.0};

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.8);

	glShadeModel(GL_SMOOTH);
}

void OpenGLRenderer::setMaterial(Vector4 color)
{
	glEnable(GL_LIGHTING);

	GLfloat ambient[] = {color.x/255 * 0.01, color.y/255 * 0.01, color.z/255 * 0.01, color.w};
	GLfloat diffuse[] = {color.x/255, color.y/255, color.z/255, color.w};
	GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat shine = 100.0;
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialf(GL_FRONT, GL_SHININESS, shine);

	GLfloat emission[] = {0.3, 0.3, 0.3, 1.0};
	glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}

void OpenGLRenderer::drawObject(GLfloat* projection, GLfloat* model_view, int markerId=0, double markerWidth=80.0)
{
	if(markerId >= 0)
	{
		// draw 3d object
		glEnable(GL_BLEND);
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(projection);

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(model_view);

		glTranslated(0.0, 0.0, markerWidth/2);
		glColor3f(0.95f, 0.95f, 0.95f); 
		glutSolidCube(markerWidth);
	}
}
