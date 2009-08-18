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

void OpenGLRenderer::drawBackground(IplImage* inputImage)
{
	// Draw background
#ifdef DRAWPIXEL_MODE
	// using Draw Pixels
	IplImage* flipImage = cvCreateImage(cvGetSize(inputImage), inputImage->depth, inputImage->nChannels);
	cvFlip(inputImage, flipImage);

	glDisable(GL_DEPTH_TEST);
//	glPixelZoom(resizeFactorX, resizeFactorY);
	glDrawPixels(flipImage->width, flipImage->height, GL_BGR_EXT, GL_UNSIGNED_BYTE, flipImage->imageData);
	glEnable(GL_DEPTH_TEST);

	cvReleaseImage(&flipImage);
#else
	// using texturemapping
	IplImage* resizeTexture = cvCreateImage(cvSize(512, 512), IPL_DEPTH_8U, 3);
	cvResize(inputImage, resizeTexture);

	glEnable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    GLuint backgroundTexture;
    glGenTextures(1, &backgroundTexture);
    glBindTexture(GL_TEXTURE_2D, backgroundTexture);

    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, resizeTexture->width, resizeTexture->width, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, resizeTexture->imageData);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//	glOrtho(0, 1, 0, 1, -1, 1);
	// isflip
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

    glDeleteTextures(1, &backgroundTexture);

	cvReleaseImage(&resizeTexture);
	glEnable(GL_DEPTH_TEST);
#endif
}

void OpenGLRenderer::setLight()
{
	GLfloat diffuse0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat ambient0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat specular0[]={1.0, 1.0, 1.0, 1.0};
	GLfloat light0_pos[]={0.0, 0.0, 200, 1.0};

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
