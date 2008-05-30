#include <cv.h>
#include <highgui.h>

#include <iostream>
using namespace std;

#include "../include/windageCalibration.h"
#include "../include/windageReconstruction.h"

void main()
{
	IplImage* image1 = cvLoadImage("../Data/captureImage2.jpg");
	IplImage* image2 = cvLoadImage("../Data/captureImage1.jpg");
	IplImage* image3 = cvLoadImage("../Data/captureImage3.jpg");

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

	SolveCalibration(&instrinsicMatrix, &distortionCoefficients, rotationVector, translationVector,
		corner, chessBoardWidth, chessBoardHeight, filedSize, image1->width, image1->height, imageCount);
	
	// get extrinsic matrix
	int i;
	for(i=0; i<imageCount; i++)
	{
		GetExtrinsicMatrix(&extrinsicMatrix[i], rotationVector[i], translationVector[i]);
	}
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

	cvNamedWindow("result1");
	cvShowImage("result1", temp1);

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
	
	// change chessboard width&height
	IplImage* resultImage1 = cvCreateImage(cvSize(filedSize*(chessBoardHeight+2), filedSize*(chessBoardWidth+2)), IPL_DEPTH_8U, 3);

	// draw reprojection
	Matrix3 rotation;
	for(y=0; y<3; y++)
	{
		for(x=0; x<3; x++)
		{
			rotation.m[y][x] = extrinsicMatrix[0].m[x][y];
		}
	}

	Vector3 originalp, resultp;
	for(y=0; y<resultImage1->height; y++)
	{
		for(x=0; x<resultImage1->width; x++)
		{
			originalp.x = x - filedSize*2;
			originalp.y = y - filedSize*2;
			originalp.z = 0;

			resultp = instrinsicMatrix * (rotation * originalp + translationVector[0]);
//			WorldToImageCoordinate(&resultp, originalp, instrinsicMatrix, rotation, rotationVector[0]);

			resultp.x /= resultp.z;
			resultp.y /= resultp.z;
			resultp.z = 1;

			if(0 < resultp.x && resultp.x < undistortion1->width && 0 < resultp.y && resultp.y < undistortion1->height)
			{
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 2] = undistortion1->imageData[(int)resultp.y*undistortion1->widthStep + (int)resultp.x*3 + 2];
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 1] = undistortion1->imageData[(int)resultp.y*undistortion1->widthStep + (int)resultp.x*3 + 1];
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 0] = undistortion1->imageData[(int)resultp.y*undistortion1->widthStep + (int)resultp.x*3 + 0];
			}
			else
			{
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 2] = (unsigned char)0;
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 1] = (unsigned char)0;
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 0] = (unsigned char)0;
			}

			if(x == filedSize*2 || y == filedSize*2)
			{
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 2] = (unsigned char)255;
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 1] = (unsigned char)0;
				resultImage1->imageData[y*resultImage1->widthStep + x*3 + 0] = (unsigned char)0;
			}
		}
	}

	cvNamedWindow("reprojection1");
	cvShowImage("reprojection1", resultImage1);

	cvReleaseImage(&undistortion1);

	cvReleaseImage(&resultImage1);

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