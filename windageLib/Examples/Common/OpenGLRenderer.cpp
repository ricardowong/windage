/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr) (wwoo@gist.ac.kr)
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

#include <cv.h>

#include "OpenGLRenderer.h"
using namespace windage;

void OpenGLRenderer::Initialize(int width, int height, char* windowName)
{
	this->width = width;
	this->height = height;
	this->cameraWidth = width;
	this->cameraHeight = height;

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutCreateWindow(windowName);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_ALPHA_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_BLEND);

	glClearColor(0.0, 0.0, 0.0, 0.0);

	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glViewport(0, 0, width, height);
	gluPerspective(60, (double)width/(double)height, 0.1, 10000.0);
}

void OpenGLRenderer::DrawClear()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRenderer::SetLight()
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

void OpenGLRenderer::SetMaterial(Vector4 color)
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

void OpenGLRenderer::DrawObject(GLfloat* projection, GLfloat* model_view, int markerId=0, double markerWidth=80.0)
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

		glDisable(GL_BLEND);
	}
}

void OpenGLRenderer::DrawAxis(double size)
{
	glLineWidth(5);
	glBegin(GL_LINES);
	{
		glColor3f(1, 0, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(size, 0, 0);

		glColor3f(0, 1, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(0, size, 0);

		glColor3f(0, 0, 1);
		glVertex3d(0, 0, 0);
		glVertex3d(0, 0, size);
	}
	glEnd();
}

void OpenGLRenderer::AttatchReference(IplImage* image)
{
	cvResize(image, this->referenceImage);
}

void OpenGLRenderer::DrawReference(double width, double height)
{
	glBindTexture(GL_TEXTURE_2D, this->referenceTexture);

	glEnable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGB, referenceImage->width, referenceImage->height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, referenceImage->imageData);

    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_POLYGON);
    {
		glTexCoord2d(0,1);
		glVertex3d(-width/2.0, -height/2.0, 0);

		glTexCoord2d(1,1);
		glVertex3d(+width/2.0, -height/2.0, 0);

		glTexCoord2d(1,0);
		glVertex3d(+width/2.0, +height/2.0, 0);

		glTexCoord2d(0,0);
		glVertex3d(-width/2.0, +height/2.0, 0);
    }
    glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);

	// outline
	glColor3f(1, 0, 1);
	glBegin(GL_LINE_STRIP);
	{
		glVertex3d(-width/2.0, -height/2.0, 0);
		glVertex3d(+width/2.0, -height/2.0, 0);
		glVertex3d(+width/2.0, +height/2.0, 0);
		glVertex3d(-width/2.0, +height/2.0, 0);
		glVertex3d(-width/2.0, -height/2.0, 0);
	}
	glEnd();
}

void OpenGLRenderer::DrawCamera(Calibration* calibration, IplImage* image)
{
	CvScalar pt = calibration->GetCameraPosition();
	CvScalar at = calibration->GetLookAt();
	CvScalar up = calibration->GetUpPoint();
	CvScalar ri = calibration->GetRightPoint();

	Vector3 cameraPosition	= Vector3(pt.val[0], pt.val[1], pt.val[2]);
	Vector3 lookAt			= Vector3(at.val[0], at.val[1], at.val[2]);
	Vector3 upVector		= Vector3(up.val[0], up.val[1], up.val[2]);
	Vector3 rightVector		= Vector3(ri.val[0], ri.val[1], ri.val[2]);
	Vector3 point[4];

	// translate relation value
	lookAt -= cameraPosition;
	upVector -= cameraPosition;
	rightVector -= cameraPosition;
	
	lookAt *= ((double)calibration->GetParameters()[0]) / lookAt.getLength();
	upVector *= ((double)this->cameraHeight/2.0) / upVector.getLength();
	rightVector *= ((double)this->cameraWidth/2.0) / rightVector.getLength();

	point[0] = cameraPosition + lookAt + upVector - rightVector;
	point[1] = cameraPosition + lookAt + upVector + rightVector;
	point[2] = cameraPosition + lookAt - upVector + rightVector;
	point[3] = cameraPosition + lookAt - upVector - rightVector;

	// translate absolution value
	lookAt += cameraPosition;
	upVector += cameraPosition;	
	rightVector += cameraPosition;

	glColor3f(1, 1, 0);
	glBegin(GL_LINES);
	{
		for(int i=0; i<4; i++)
		{
			glVertex3f(cameraPosition.x, cameraPosition.y, cameraPosition.z);
			glVertex3f(point[i].x, point[i].y, point[i].z);
		}
	}
	glEnd();

	glBegin(GL_LINE_STRIP);
	{
		for(int i=0; i<4; i++)
			glVertex3f(point[i].x, point[i].y, point[i].z);
		glVertex3f(point[0].x, point[0].y, point[0].z);
	}
	glEnd();

	// draw input image
	cvResize(image, this->inputImage);
	glBindTexture(GL_TEXTURE_2D, this->inputTexture);

	glEnable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glDisable(GL_LIGHTING);

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGB, inputImage->width, inputImage->height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, inputImage->imageData);

	glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_POLYGON);
    {
		glTexCoord2d(0,0);
		glVertex3d(point[0].x, point[0].y, point[0].z);

		glTexCoord2d(1,0);
		glVertex3d(point[1].x, point[1].y, point[1].z);

		glTexCoord2d(1,1);
		glVertex3d(point[2].x, point[2].y, point[2].z);

		glTexCoord2d(0,1);
		glVertex3d(point[3].x, point[3].y, point[3].z);
    }
    glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
}

