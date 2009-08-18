#ifndef OPENGL_RENDERER_H
#define OPENGL_RENDERER_H

#include <GL/glut.h>
#include <cv.h>

#include "Utils/wVector.h"
using namespace windage;

//#define DRAWPIXEL_MODE

class OpenGLRenderer
{
public:
	static void init(int width = 320, int height = 240);
//	static void idle();
//	static void display();

	static void setLight();
	static void setMaterial(Vector4 color);

	static void drawClear();
	static void drawBackground(IplImage* inputImage);
	static void drawObject(GLfloat* projection, GLfloat* model_view, int markerId, double markerWidth);

	static void mouse(int iButton, int iState, int x, int y);
	static void motion(int x, int y);

	static int cursorX;
	static int cursorY;

	enum MODE{RENDER=1, CLICKED=2};
	static MODE mode;

private:
	static int windowWidth;
	static int windowHeight;

	static int cameraWidth;
	static int cameraHeight;
};

#endif