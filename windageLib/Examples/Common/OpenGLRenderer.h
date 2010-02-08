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

#ifndef OPENGL_RENDERER_H
#define OPENGL_RENDERER_H

#include <GL/glut.h>
#include <cv.h>
#include "Structures/Vector.h"
#include "Structures/Calibration.h"

class OpenGLRenderer
{
private:
	static const int TEXTURE_SIZE = 512;
	GLuint referenceTexture;
	GLuint inputTexture;
	IplImage* referenceImage;
	IplImage* inputImage;

	int width;
	int height;
	int cameraWidth;
	int cameraHeight;

public:
	OpenGLRenderer()
	{
		referenceImage = cvCreateImage(cvSize(TEXTURE_SIZE, TEXTURE_SIZE), IPL_DEPTH_8U, 3);
		inputImage = cvCreateImage(cvSize(TEXTURE_SIZE, TEXTURE_SIZE), IPL_DEPTH_8U, 3);

		cvZero(referenceImage);
		cvZero(inputImage);

		glGenTextures(1, &referenceTexture);
		glGenTextures(1, &inputTexture);
	}
	~OpenGLRenderer()
	{
		if(this->referenceImage) cvReleaseImage(&referenceImage);
		referenceImage = NULL;
		if(this->inputImage) cvReleaseImage(&inputImage);
		inputImage = NULL;

		if(referenceTexture) glDeleteTextures(1, &referenceTexture);
		referenceTexture = 0;
		if(inputTexture) glDeleteTextures(1, &inputTexture);
		inputTexture = 0;
	}

	void Initialize(int width = 320, int height = 240, char * windowName = "OpenGL Renderer");

	void SetLight();
	void SetMaterial(windage::Vector4 color);

	void DrawClear();
	void DrawBackground(IplImage* inputImage);
	void DrawObject(GLfloat* projection, GLfloat* model_view, int markerId, double markerWidth);

	inline void SetCameraSize(int width, int height){this->cameraWidth = width, this->cameraHeight = height;};
	void DrawAxis(double size);
	void AttatchReference(IplImage* image);
	void DrawReference(double width, double height);
	void DrawCamera(windage::Calibration* calibration, IplImage* image = NULL);
	void DrawCameraAxis(windage::Calibration* calibration, double size = 1.0);

	// overwriting
	/*
	virtual void idle()
	{
	}
	virtual static void display()
	{
	}
	virtual void mouse(int iButton, int iState, int x, int y)
	{
	}
	virtual void motion(int x, int y)
	{
	}
	*/
};

#endif