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

#include "Calibration.h"
#include "Utils/wMatrix.h"
using namespace windage;

Calibration::Calibration()
{
	intrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
	distortionCoefficients = cvCreateMat(4, 1, CV_64FC1);
	extrinsicMatrix = cvCreateMat(4, 4, CV_64FC1);

	cvmSetZero(intrinsicMatrix);
	cvmSetZero(distortionCoefficients);
	cvmSetZero(extrinsicMatrix);

	dstMapX = NULL;
	dstMapY = NULL;

//	chessboardPoints = NULL;
}

Calibration::~Calibration()
{
	this->Release();
}

void Calibration::Release()
{
	if(intrinsicMatrix)			cvReleaseMat(&intrinsicMatrix);
	if(distortionCoefficients)	cvReleaseMat(&distortionCoefficients);
	if(extrinsicMatrix)			cvReleaseMat(&extrinsicMatrix);

	if(dstMapX) cvReleaseImage(&dstMapX);
	dstMapX = NULL;
	if(dstMapY) cvReleaseImage(&dstMapY);
	dstMapY = NULL;
}

void Calibration::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4)
{
	this->SetIntrinsicMatirx(fx, fy, cx, cy);
	this->SetDistortionCoefficients(d1, d2, d3, d4);
}

void Calibration::SetIntrinsicMatirx(double fx, double fy, double cx, double cy)
{
	cvmSetZero(intrinsicMatrix);
	cvmSet(intrinsicMatrix, 0, 0, fx);
	cvmSet(intrinsicMatrix, 1, 1, fy);
	cvmSet(intrinsicMatrix, 0, 2, cx);
	cvmSet(intrinsicMatrix, 1, 2, cy);
	cvmSet(intrinsicMatrix, 2, 2, 1);
}

void Calibration::SetDistortionCoefficients(double d1, double d2, double d3, double d4)
{
	cvmSetZero(distortionCoefficients);
	cvmSet(distortionCoefficients, 0, 0, d1);
	cvmSet(distortionCoefficients, 1, 0, d2);
	cvmSet(distortionCoefficients, 2, 0, d3);
	cvmSet(distortionCoefficients, 3, 0, d4);
}

void Calibration::SetExtrinsicMatrix(CvMat* matrix)
{
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
		{
			cvSetReal2D(this->extrinsicMatrix, y, x, cvGetReal2D(matrix, y, x));
		}
	}
}
void Calibration::SetExtrinsicMatrix(float* matrix)
{
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
		{
			cvSetReal2D(this->extrinsicMatrix, y, x, (double)matrix[y*4+x]);
		}
	}
}

void Calibration::SetExtrinsicMatrix(double* matrix)
{
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
		{
			cvSetReal2D(this->extrinsicMatrix, y, x, matrix[y*4+x]);
		}
	}
}

void Calibration::ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector)
{
	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);

	cvSetZero(extrinsicMatrix);
	int i, j;
    for(i=0; i < 3; i++) 
    {
		cvSetReal2D(extrinsicMatrix, i, 3, cvGetReal1D(translationVector, i));
    } 

	cvRodrigues2(rotationVector, rotationMatrix);
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			cvSetReal2D(extrinsicMatrix, i, j, cvGetReal2D(rotationMatrix, i, j)); 
		}
	}

	cvSetReal2D(extrinsicMatrix, 3, 3, 1.0);

	cvReleaseMat(&rotationMatrix);
}

void Calibration::GetCameraPosition(CvMat* output)
{
	cvSetReal1D(output, 0, cvGetReal2D(this->extrinsicMatrix, 0, 3));
	cvSetReal1D(output, 1, cvGetReal2D(this->extrinsicMatrix, 1, 3));
	cvSetReal1D(output, 2, cvGetReal2D(this->extrinsicMatrix, 2, 3));
}

CvScalar Calibration::GetCameraPosition()
{
	windage::Matrix3 rotation;
	rotation.m[0][0] = cvGetReal2D(this->extrinsicMatrix, 0, 0);
	rotation.m[0][1] = cvGetReal2D(this->extrinsicMatrix, 1, 0);
	rotation.m[0][2] = cvGetReal2D(this->extrinsicMatrix, 2, 0);

	rotation.m[1][0] = cvGetReal2D(this->extrinsicMatrix, 0, 1);
	rotation.m[1][1] = cvGetReal2D(this->extrinsicMatrix, 1, 1);
	rotation.m[1][2] = cvGetReal2D(this->extrinsicMatrix, 2, 1);

	rotation.m[2][0] = cvGetReal2D(this->extrinsicMatrix, 0, 2);
	rotation.m[2][1] = cvGetReal2D(this->extrinsicMatrix, 1, 2);
	rotation.m[2][2] = cvGetReal2D(this->extrinsicMatrix, 2, 2);

	windage::Vector3 translation;
	translation.x = cvGetReal2D(this->extrinsicMatrix, 0, 3);
	translation.y = cvGetReal2D(this->extrinsicMatrix, 1, 3);
	translation.z = cvGetReal2D(this->extrinsicMatrix, 2, 3);

	rotation = rotation.Transpose();
	windage::Vector3 temp = rotation * translation;

	CvScalar cameraPos;

	cameraPos.val[0] = temp.x;
	cameraPos.val[1] = temp.y;
	cameraPos.val[2] = temp.z;

//	cameraPos.val[0] = cvGetReal2D(this->extrinsicMatrix, 0, 3);
//	cameraPos.val[1] = cvGetReal2D(this->extrinsicMatrix, 1, 3);
//	cameraPos.val[2] = cvGetReal2D(this->extrinsicMatrix, 2, 3);
	return cameraPos;
}

int Calibration::ConvertWorld2Camera(CvMat* output, CvMat* input)
{
	cvMatMul(this->extrinsicMatrix, input, output);
	return 1;
}

int Calibration::ConvertCamera2Image(CvMat* output, CvMat* input)
{
	cvMatMul(this->intrinsicMatrix, input, output);
	return 1;
}

int Calibration::ConvertWorld2Image(CvMat* output, CvMat* input)
{
	CvMat* cameraCoordinatePosition = cvCreateMat(4, 1, CV_64FC1);
	ConvertWorld2Camera(cameraCoordinatePosition, input);

	CvMat* cameraCoordinate3DPosition = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(cameraCoordinate3DPosition, 0, cvGetReal1D(cameraCoordinatePosition, 0));
	cvSetReal1D(cameraCoordinate3DPosition, 1, cvGetReal1D(cameraCoordinatePosition, 1));
	cvSetReal1D(cameraCoordinate3DPosition, 2, cvGetReal1D(cameraCoordinatePosition, 2));

	ConvertCamera2Image(output, cameraCoordinate3DPosition);

	cvSetReal1D(output, 0, cvGetReal1D(output, 0) / cvGetReal1D(output, 2));
	cvSetReal1D(output, 1, cvGetReal1D(output, 1) / cvGetReal1D(output, 2));
	cvSetReal1D(output, 2, 1.0);

	cvReleaseMat(&cameraCoordinatePosition);
	cvReleaseMat(&cameraCoordinate3DPosition);

	return 1;
}

CvPoint Calibration::ConvertWorld2Image(double x, double y, double z)
{
	CvMat* imageCoordinate = cvCreateMat(3, 1,CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(4, 1,CV_64FC1);

	cvSetReal1D(worldCoordinate, 0, x);
	cvSetReal1D(worldCoordinate, 1, y);
	cvSetReal1D(worldCoordinate, 2, z);
	cvSetReal1D(worldCoordinate, 3, 1);
	ConvertWorld2Image(imageCoordinate, worldCoordinate);

	CvPoint point;
	point.x = cvGetReal1D(imageCoordinate, 0);
	point.y = cvGetReal1D(imageCoordinate, 1);

	cvReleaseMat(&imageCoordinate);
	cvReleaseMat(&worldCoordinate);

	return point;
}


int Calibration::ConvertCamera2World(CvMat* output, CvMat* input)
{
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(translationVector, 0, cvGetReal2D(this->extrinsicMatrix, 0, 3));
	cvSetReal1D(translationVector, 1, cvGetReal2D(this->extrinsicMatrix, 1, 3));
	cvSetReal1D(translationVector, 2, cvGetReal2D(this->extrinsicMatrix, 2, 3));

	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			cvSetReal2D(rotationMatrix, x, y, cvGetReal2D(this->extrinsicMatrix, x, y));
		}
	}

	CvMat* inversRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(rotationMatrix, inversRotationMatrix);

	cvMatMul(inversRotationMatrix, translationVector, translationVector);

	CvMat* temp = cvCreateMat(3, 1, CV_64FC1);
	cvSub(input, translationVector, temp);

	cvMatMul(inversRotationMatrix, temp, output);

	cvReleaseMat(&temp);

	cvReleaseMat(&inversRotationMatrix);
	cvReleaseMat(&rotationMatrix);
	cvReleaseMat(&translationVector);

	return 1;
}

int Calibration::ConvertImage2Camera(CvMat* output, CvMat* input, double z)
{
	double rho;

	CvMat* inversIntrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(this->intrinsicMatrix, inversIntrinsicMatrix);

	CvMat* temp = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversIntrinsicMatrix, input, temp);

	rho = z / cvGetReal1D(temp, 2);

	cvSetReal1D(output, 0, cvGetReal1D(temp, 0) * rho);
	cvSetReal1D(output, 1, cvGetReal1D(temp, 1) * rho);
	cvSetReal1D(output, 2, cvGetReal1D(temp, 2) * rho);

	cvReleaseMat(&temp);
	cvReleaseMat(&inversIntrinsicMatrix);

	return 1;
}

int Calibration::ConvertImage2World(CvMat* output, CvMat* input, double z)
{
	CvMat* inversIntrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(this->intrinsicMatrix, inversIntrinsicMatrix);

	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(translationVector, 0, cvGetReal2D(this->extrinsicMatrix, 0, 3));
	cvSetReal1D(translationVector, 1, cvGetReal2D(this->extrinsicMatrix, 1, 3));
	cvSetReal1D(translationVector, 2, cvGetReal2D(this->extrinsicMatrix, 2, 3));

	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			cvSetReal2D(rotationMatrix, x, y, cvGetReal2D(this->extrinsicMatrix, x, y));
		}
	}

//	CvMat* inversRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
//	cvInvert(rotationMatrix, inversRotationMatrix);

	CvMat* imageDot = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversIntrinsicMatrix, input, imageDot);

	CvMat* tempMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvSetReal2D(tempMatrix, 0, 0, - cvGetReal1D(imageDot, 0)); cvSetReal2D(tempMatrix, 0, 1, cvGetReal2D(rotationMatrix, 0, 0)); cvSetReal2D(tempMatrix, 0, 2, cvGetReal2D(rotationMatrix, 0, 1));
	cvSetReal2D(tempMatrix, 1, 0, - cvGetReal1D(imageDot, 1)); cvSetReal2D(tempMatrix, 1, 1, cvGetReal2D(rotationMatrix, 1, 0)); cvSetReal2D(tempMatrix, 1, 2, cvGetReal2D(rotationMatrix, 1, 1));
	cvSetReal2D(tempMatrix, 2, 0, - cvGetReal1D(imageDot, 2)); cvSetReal2D(tempMatrix, 2, 1, cvGetReal2D(rotationMatrix, 2, 0)); cvSetReal2D(tempMatrix, 2, 2, cvGetReal2D(rotationMatrix, 2, 1));

	CvMat* inversTempMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(tempMatrix, inversTempMatrix);

	CvMat* tempVector = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(tempVector, 0, - (cvGetReal2D(rotationMatrix, 0, 2) * z + cvGetReal1D(translationVector, 0)) );
	cvSetReal1D(tempVector, 1, - (cvGetReal2D(rotationMatrix, 1, 2) * z + cvGetReal1D(translationVector, 1)) );
	cvSetReal1D(tempVector, 2, - (cvGetReal2D(rotationMatrix, 2, 2) * z + cvGetReal1D(translationVector, 2)) );

	CvMat* tempResult = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversTempMatrix, tempVector, tempResult);

	double rho = cvGetReal1D(tempResult, 0);

	cvSetReal1D(output, 0, cvGetReal1D(tempResult, 1));
	cvSetReal1D(output, 1, cvGetReal1D(tempResult, 2));
	cvSetReal1D(output, 2, z);

	cvReleaseMat(&imageDot);
	cvReleaseMat(&tempMatrix);
	cvReleaseMat(&inversTempMatrix);
	cvReleaseMat(&tempVector);

	cvReleaseMat(&tempResult);

//	cvReleaseMat(&inversRotationMatrix);
	cvReleaseMat(&translationVector);
	cvReleaseMat(&rotationMatrix);
	cvReleaseMat(&inversIntrinsicMatrix);

	return 1;
}

CvPoint2D64f Calibration::ConvertImage2World(double ix, double iy, double wz)
{
	CvMat* imageCoordinate = cvCreateMat(3, 1,CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(3, 1,CV_64FC1);

	cvSetReal1D(imageCoordinate, 0, ix);
	cvSetReal1D(imageCoordinate, 1, iy);
	cvSetReal1D(imageCoordinate, 2, 1);
	ConvertImage2World(worldCoordinate, imageCoordinate, wz);

	CvPoint2D64f point;
	point.x = cvGetReal1D(worldCoordinate, 0);
	point.y = cvGetReal1D(worldCoordinate, 1);

	cvReleaseMat(&imageCoordinate);
	cvReleaseMat(&worldCoordinate);

	return point;
}

void Calibration::InitUndistortionMap(int width, int height)
{
	if(dstMapX) cvReleaseImage(&dstMapX);
	if(dstMapY) cvReleaseImage(&dstMapY);

	dstMapX = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
	dstMapY = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);

	cvInitUndistortMap(this->intrinsicMatrix,this->distortionCoefficients, this->dstMapX, this->dstMapY);
}

void Calibration::Undistortion(IplImage* input, IplImage* output)
{
	if(dstMapX && dstMapY)
		cvRemap(input, output, this->dstMapX, this->dstMapY);
	else
		cvUndistort2(input, output, this->intrinsicMatrix, this->distortionCoefficients);
}

void Calibration::DrawInfomation(IplImage* colorImage, double size)
{
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(size, 0.0, 0.0), CV_RGB(255, 0, 0), 2);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, size, 0.0), CV_RGB(0, 255, 0), 2);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, 0.0, size), CV_RGB(0, 0, 255), 2);
}