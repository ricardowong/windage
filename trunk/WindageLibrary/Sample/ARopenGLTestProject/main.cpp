#include <cv.h>
#include <highgui.h>

#include <gl/glut.h>

#include <iostream>
using namespace std;

#include "../../include/windageCalibration.h"
#include "../../include/windageAugmentedReality.h"

IplImage* background;
Matrix3 internalMatrix;
Matrix3 rotationMatrix;
Vector3 translateVector;
int width, height;

void display()
{
	// check if there have been any openGL problems
    GLenum errCode = glGetError();
    if( errCode != GL_NO_ERROR )
    {
        const GLubyte *errString = gluErrorString( errCode );
        fprintf( stderr, "OpenGL error: %s\n", errString );
    }

	 // clear the buffers of the last frame
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	// render virtual objects
	bool isFlip = false;
	DrawBackgroundTexture(background, background->width, isFlip);
	SetOpenGLCalibrationData(internalMatrix, rotationMatrix, translateVector, width, height, isFlip);

	GLfloat colorRed[3] = { 1.0, 0.0, 0.0 };
	GLfloat colorGreen[3] = { 0.0, 1.0, 0.0 };
	GLfloat colorBlue[3] = { 0.0, 0.0, 1.0 };
	glDisable(GL_LIGHTING);
	glLineWidth( 3 );
	glBegin(GL_LINES);
		glMaterialfv(GL_FRONT, GL_AMBIENT, colorRed);
		glColor3fv( colorRed );
		glVertex3f(0, 0, 0);
		glVertex3f(100, 0, 0);
		glMaterialfv(GL_FRONT, GL_AMBIENT, colorGreen);
		glColor3fv( colorGreen );
		glVertex3f(0, 0, 0);
		glVertex3f(0, 100, 0);
		glMaterialfv(GL_FRONT, GL_AMBIENT, colorBlue);
		glColor3fv( colorBlue );
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 100);
	glEnd();
	glPushMatrix();
		glColor3fv( colorRed );
		glTranslatef( 100, 0, 0 );
		glRotatef( 90, 0, 1, 0 );
		glutSolidCone( 5, 10, 16, 16 );
	glPopMatrix();
	glPushMatrix();
		glColor3fv( colorGreen );
		glTranslatef( 0, 100, 0 );
		glRotatef( -90, 1, 0, 0 );
		glutSolidCone( 5, 10, 16, 16 );
	glPopMatrix();
	glPushMatrix();
		glColor3fv( colorBlue );
		glTranslatef( 0, 0, 100 );
		glRotatef( 90, 0, 0, 1 );
		glutSolidCone( 5, 10, 16, 16 );
	glPopMatrix();
	glEnable(GL_LIGHTING);

	glutSwapBuffers();
}

void init()
{
    // properly scale normal vectors
    glEnable( GL_NORMALIZE );

    // turn on default lighting
    glEnable( GL_LIGHTING );

    // light 0
    GLfloat light_position[] = { 100.0, 500, 200, 1.0 };
    GLfloat white_light[] = { 1.0, 1.0, 1.0, 0.8 };
    GLfloat lmodel_ambient[] = { 0.9, 0.9, 0.9, 0.5 };

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

    glEnable(GL_LIGHT0);

    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_TEXTURE_2D);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);

    // Callback functions
    glutDisplayFunc( display );
}

void main()
{
	IplImage* image1 = cvLoadImage("../../Data/captureImage1.jpg");
	IplImage* image2 = cvLoadImage("../../Data/captureImage2.jpg");
	IplImage* image3 = cvLoadImage("../../Data/captureImage3.jpg");

	IplImage* temp1 = cvCloneImage(image1);

	// change chessboard width&height
	int chessBoardWidth = 7;
	int chessBoardHeight = 10;

	int imageCount = 3;
	int pointCount = (chessBoardWidth-1) * (chessBoardHeight-1);
	int filedSize = 26;

/*** calculate start ***/
	// find chess board
	CvPoint2D32f * corner = new CvPoint2D32f[imageCount * pointCount];
	FindChessBoardCorner(corner, image1, chessBoardWidth, chessBoardHeight);
	FindChessBoardCorner(corner + pointCount, image2, chessBoardWidth, chessBoardHeight);
	FindChessBoardCorner(corner + pointCount * 2, image3, chessBoardWidth, chessBoardHeight);

	// solve calibration
	Matrix3 instrinsicMatrix;
	Vector4 distortionCoefficients;
	Vector3* rotationVector = new Vector3[imageCount];
	Vector3* translationVector = new Vector3[imageCount];
	Matrix4* extrinsicMatrix = new Matrix4[imageCount];

	SolveCalibration2(&instrinsicMatrix, &distortionCoefficients, rotationVector, translationVector,
		corner, chessBoardWidth, chessBoardHeight, filedSize, image1->width, image1->height, imageCount);
	
	// get extrinsic matrix
	int i;
	for(i=0; i<imageCount; i++)
	{
		GetExtrinsicMatrix(&extrinsicMatrix[i], rotationVector[i], translationVector[i]);
	}

	UpdateExtrinsicParams2(&rotationVector[0], &translationVector[0],
		instrinsicMatrix, distortionCoefficients, corner, chessBoardWidth, chessBoardHeight, filedSize, image1->width, image1->height);
/*** calculate end ***/

/*** draw information start ***/
	// draw find chess board
	int count=(chessBoardHeight-1)*(chessBoardWidth-1);
	for(i=0; i<count-1; i++)
	{
		cvCircle(temp1, cvPoint((int)corner[i].x, (int)corner[i].y), 3, CV_RGB(255 * (count-i)/count, 255 * i/count, 0), CV_FILLED);
		cvLine(temp1, cvPoint((int)corner[i].x, (int)corner[i].y), cvPoint((int)corner[i+1].x, (int)corner[i+1].y), CV_RGB(255 * (count-i)/count, 255 * i/count, 0));
	}
	cvCircle(temp1, cvPoint((int)corner[i].x, (int)corner[i].y), 3, CV_RGB(255 * (count-i)/count, 255 * i/count, 0), CV_FILLED);

	// printout calibration data
	int x, y;
	cout << "/*** Intrinsic Matrix ***/" << endl;
	for(y=0; y<3; y++)
	{
		for(x=0; x<3; x++)
		{
			cout << instrinsicMatrix.m[y][x] << ", ";
		}
		cout << endl;
	}
	cout << endl;

	cout << "/*** Radial Distortion Coefficients ***/" << endl;
	cout << distortionCoefficients.x << ", ";
	cout << distortionCoefficients.y << ", ";
	cout << distortionCoefficients.z << ", ";
	cout << distortionCoefficients.w << ", ";
	cout << endl << endl;

	cout << "/*** Translation & Rotation Vectors ***/" << endl;

	cout << "Image : 1" << endl;
	cout << ">> Translation Vector : ";
	for(x=0; x<3; x++)
	{
		cout << translationVector[0].v[x] << ", ";
	}
	cout << endl;

	cout << ">> Rotation Vector : ";
	for(x=0; x<3; x++)
	{
		cout << rotationVector[0].v[x] << ", ";
	}
	cout << endl;

	cout << ">> Extrinsic Matrix" << endl;
	for(y=0; y<4; y++)
	{
		for(x=0; x<4; x++)
		{
			cout << extrinsicMatrix[0].m[y][x] << ", ";
		}
		cout << endl;
	}
	cout << endl;

	// radial Undistortion
	IplImage* undistortion1 = cvCreateImage(cvGetSize(image1), IPL_DEPTH_8U, 3);
	UnRadialDistortion(undistortion1, image1, instrinsicMatrix, distortionCoefficients);

	background = cvCreateImage(cvSize(512, 512), IPL_DEPTH_8U, 3);
	cvResize(image1, background);

	internalMatrix = instrinsicMatrix;
	rotationMatrix = Matrix3(	extrinsicMatrix[0]._11, extrinsicMatrix[0]._12, extrinsicMatrix[0]._13,
											extrinsicMatrix[0]._21, extrinsicMatrix[0]._22, extrinsicMatrix[0]._23,
											extrinsicMatrix[0]._31, extrinsicMatrix[0]._32, extrinsicMatrix[0]._33);
	translateVector = translationVector[0];
	width = image1->width;
	height = image1->height;

	// OpenGL loop
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glutInitWindowPosition( 100, 100 );

	glutInitWindowSize(image1->width, image1->height);
	glutCreateWindow( "windage Demo Application" );

	init();
	glutMainLoop();

	cvReleaseImage(&undistortion1);
	cvReleaseImage(&temp1);
/*** draw information end ***/

	// release memory
	cvReleaseImage(&image1);
	cvReleaseImage(&image2);
	cvReleaseImage(&image3);

	delete[] corner;

	delete[] rotationVector;
	delete[] translationVector;

	delete[] extrinsicMatrix;
	
	cvWaitKey(0);
}