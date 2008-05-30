
#include <cv.h>
#include <gl/glut.h>

#include "../include/windageMatrix.h"


extern "C" __declspec(dllexport)
void DrawBackgroundTexture(IplImage* image, int imageWidth, bool isFlip)
{
	// Draw OpenGL Background	
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	glTexImage2D(GL_TEXTURE_2D, 0, 3, imageWidth, imageWidth, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image->imageData);

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
}

extern "C" __declspec(dllexport)
void SetOpenGLCalibrationData(Matrix3 internalMatrix, Matrix3 rotationMatrix, Vector3 translationVector, int imageWidth, int imageHeight, bool flip)
{
	// projection matrix setting...
	GLdouble m[16];
	int i, j;
	for(i=0; i<16; ++i)
		m[i] = 0;

	double w = imageWidth;
	double h = imageHeight;
	double f = internalMatrix._11;
	double g = internalMatrix._22;
	double s = 0;
	double cx = internalMatrix._13;
	double cy = internalMatrix._23;
	double _near = f/16.0f;
	double _far = f*16.0f;

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
	glLoadMatrixd(m);
	
	if(flip)
		glScalef(1, -1, 1);

	// modelview matrix setting...
	for(i=0; i<3; ++i)
	{
		for(j=0; j<3; ++j)
		{
			m[i+j*4] = rotationMatrix.m[j][i];
		}
	}
	m[0+3*4] = translationVector.x;
	m[1+3*4] = translationVector.y;
	m[2+3*4] = translationVector.z;

	m[3+0*4] = m[3+1*4] = m[3+2*4] = 0;
	m[3+3*4] = 1;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glScalef(1, 1, -1);
	glMultMatrixd(m);
}