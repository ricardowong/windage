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

#include "Coordinator/ARForOpenGL.h"
using namespace windage;
using namespace windage::Coordinator;

void ARForOpenGL::Release()
{
	if(this->textureRepository) cvReleaseImage(&textureRepository);
	textureRepository = NULL;
	if(backgroundTexture) glDeleteTextures(1, &backgroundTexture);
	backgroundTexture = 0;
}

void ARForOpenGL::Initialize(int imageWidth, int imageHeight, bool isFlip, int textureWidth)
{
	this->Release();

	this->imageWidth = imageWidth;
	this->imageHeight = imageHeight;
	this->isFlip = isFlip;
	this->textureWidth = textureWidth;

	textureRepository = cvCreateImage(cvSize(textureWidth, textureWidth), IPL_DEPTH_8U, 3);
	glGenTextures(1, &backgroundTexture);
    glBindTexture(GL_TEXTURE_2D, backgroundTexture);
}

void ARForOpenGL::DrawBackgroundTexture(IplImage* inputImage)
{
	if(inputImage->nChannels == 3)
	{
		cvResize(inputImage, this->textureRepository);
	}
	else
	{
		IplImage* tempImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 3);
		switch(inputImage->nChannels)
		{
		case 1:
			cvCvtColor(inputImage, tempImage, CV_GRAY2BGR);
			break;
		case 4:
			cvCvtColor(inputImage, tempImage, CV_BGRA2BGR);
			break;
		default:
			return;
			break;
		}
		cvResize(tempImage, this->textureRepository);
		cvReleaseImage(&tempImage);
	}

	glEnable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGB, textureRepository->width, textureRepository->height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, textureRepository->imageData);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // draw captured image texture...
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(isFlip == false)
            glOrtho(0, 1, 0, 1, -1, 1);
    else
            glOrtho(0, 1, 1, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_POLYGON);
    {
            glTexCoord2d(0,1);
            glVertex2d(0,1);

            glTexCoord2d(1,1);
            glVertex2d(1,1);

            glTexCoord2d(1,0);
            glVertex2d(1,0);

            glTexCoord2d(0,0);
            glVertex2d(0,0);
    }
    glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
}

void ARForOpenGL::SetProjectionMatrix()
{
	GLdouble* m = (GLdouble*)this->projectionMatrix.m1;
    for(int i=0; i<16; ++i)
		m[i] = 0;

    double w = imageWidth;
    double h = imageHeight;
	double f = CV_MAT_ELEM((*this->cameraParameter->GetIntrinsicMatrix()), double, 0, 0);
    double g = CV_MAT_ELEM((*this->cameraParameter->GetIntrinsicMatrix()), double, 1, 1);
    double s = 0;
    double cx = CV_MAT_ELEM((*this->cameraParameter->GetIntrinsicMatrix()), double, 0, 2);
    double cy = CV_MAT_ELEM((*this->cameraParameter->GetIntrinsicMatrix()), double, 1, 2);
    double _near = f/32.0f;
    double _far = f*32.0f;

	// original flip...
    m[0 + 0*4] = 2*f/w;
    m[0 + 1*4] = 2*s/w;
    m[0 + 2*4] = (-2*cx/w + 1);
    m[1 + 1*4] = 2*g/h;
    m[1 + 2*4] = (-2*cy/h+1);
    m[2 + 2*4] = -(_far+_near)/(_far-_near);
    m[2 + 3*4] = -2*_far*_near/(_far-_near);
    m[3 + 2*4] = -1;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(isFlip) glScalef(1.0f, -1.0f, 1.0f);
	glMultMatrixd(m);
}

void ARForOpenGL::SetModelViewMatrix()
{
	GLdouble* m = (GLdouble*)this->modelviewMatrix.m1;
    int i, j;
    for(i=0; i<16; ++i)
            m[i] = 0;

	for(j=0; j<3; ++j)
	{
		for(i=0; i<3; ++i)
		{
			m[j*4+i] = CV_MAT_ELEM((*this->cameraParameter->GetExtrinsicMatrix()), double, i, j);
		}
    }
    m[0+3*4] = CV_MAT_ELEM((*this->cameraParameter->GetExtrinsicMatrix()), double, 0, 3);
    m[1+3*4] = CV_MAT_ELEM((*this->cameraParameter->GetExtrinsicMatrix()), double, 1, 3);
    m[2+3*4] = CV_MAT_ELEM((*this->cameraParameter->GetExtrinsicMatrix()), double, 2, 3);

    m[3+0*4] = m[3+1*4] = m[3+2*4] = 0;
    m[3+3*4] = 1;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(1, 1, -1);
    glMultMatrixd(m);
}