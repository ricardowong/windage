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

#include "Structures/Matrix.h"
#include "Structures/Calibration.h"
using namespace windage;

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

	this->parameter[0] = fx;
	this->parameter[1] = fy;
	this->parameter[2] = cx;
	this->parameter[3] = cy;
}

void Calibration::SetDistortionCoefficients(double d1, double d2, double d3, double d4)
{
	cvmSetZero(distortionCoefficients);
	cvmSet(distortionCoefficients, 0, 0, d1);
	cvmSet(distortionCoefficients, 1, 0, d2);
	cvmSet(distortionCoefficients, 2, 0, d3);
	cvmSet(distortionCoefficients, 3, 0, d4);

	this->parameter[4] = d1;
	this->parameter[5] = d2;
	this->parameter[6] = d3;
	this->parameter[7] = d4;
}

void Calibration::SetExtrinsicMatrix(CvMat* matrix)
{
	cvCopy(matrix, this->extrinsicMatrix);
}
void Calibration::SetExtrinsicMatrix(float* matrix)
{
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
		{
			CV_MAT_ELEM((*this->extrinsicMatrix), double, y, x) = (double)matrix[y*4+x];
		}
	}
}

void Calibration::SetExtrinsicMatrix(double* matrix)
{
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
		{
			CV_MAT_ELEM((*this->extrinsicMatrix), double,  y, x) = matrix[y*4+x];
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
		CV_MAT_ELEM((*extrinsicMatrix), double, i, 3) = CV_MAT_ELEM((*translationVector), double, i, 0);
    } 

	cvRodrigues2(rotationVector, rotationMatrix);
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			CV_MAT_ELEM((*extrinsicMatrix), double, i, j) = CV_MAT_ELEM((*rotationMatrix), double, i, j);
		}
	}

	CV_MAT_ELEM((*extrinsicMatrix), double, 3, 3) = 1.0;

	cvReleaseMat(&rotationMatrix);
}

CvScalar Calibration::GetCameraPosition()
{
	CvScalar cameraPos;

	windage::Matrix3 rotation;
	rotation.m[0][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 0);
	rotation.m[0][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 0);
	rotation.m[0][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 0);

	rotation.m[1][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 1);
	rotation.m[1][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 1);
	rotation.m[1][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 1);

	rotation.m[2][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 2);
	rotation.m[2][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 2);
	rotation.m[2][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 2);

	windage::Vector3 translation;
	translation.x = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 3);
	translation.y = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 3);
	translation.z = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 3);

	translation = -translation;
	windage::Vector3 temp = rotation * translation;

	cameraPos.val[0] = temp.x;
	cameraPos.val[1] = temp.y;
	cameraPos.val[2] = temp.z;

	return cameraPos;
}

void Calibration::SetCameraPosition(CvScalar position)
{
	windage::Matrix3 rotation;
	rotation.m[0][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 0);
	rotation.m[0][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 0);
	rotation.m[0][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 0);

	rotation.m[1][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 1);
	rotation.m[1][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 1);
	rotation.m[1][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 1);

	rotation.m[2][0] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 2);
	rotation.m[2][1] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 2);
	rotation.m[2][2] = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 2);

	windage::Vector3 translation;
	translation.x = position.val[0];
	translation.y = position.val[1];
	translation.z = position.val[2];

	rotation = rotation.Transpose();
	translation = rotation * translation;
	translation = -translation;

	CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 3) = translation.x;
	CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 3) = translation.y;
	CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 3) = translation.z;
}

CvScalar Calibration::GetLookAt()
{
	CvScalar lookat = this->ConvertCamera2World(0.0, 0.0, 100.0);
	return lookat;
}

CvScalar Calibration::GetUpPoint()
{
	CvScalar uppoint = this->ConvertCamera2World(0.0, -100.0, 0.0);
	return uppoint;
}

CvScalar Calibration::GetRightPoint()
{
	CvScalar rightpoint = this->ConvertCamera2World(100.0, 0.0, 0.0);
	return rightpoint;
}


int Calibration::ConvertWorld2Camera(CvMat* output, CvMat* input)
{
	cvMatMul(this->extrinsicMatrix, input, output);
	return 1;
}

windage::Vector4 Calibration::ConvertWorld2Camerad(windage::Vector4 input)
{
	CvMat* cameraCoordinate = cvCreateMat(4, 1,CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(4, 1,CV_64FC1);

	CV_MAT_ELEM((*worldCoordinate), double, 0, 0) = input.x;
	CV_MAT_ELEM((*worldCoordinate), double, 1, 0) = input.y;
	CV_MAT_ELEM((*worldCoordinate), double, 2, 0) = input.z;
	CV_MAT_ELEM((*worldCoordinate), double, 3, 0) = input.w;
	ConvertWorld2Camera(cameraCoordinate, worldCoordinate);

	double ww = 1.0 / CV_MAT_ELEM((*cameraCoordinate), double, 3, 0);
	windage::Vector4 point;
	point.x = CV_MAT_ELEM((*cameraCoordinate), double, 0, 0) * ww;
	point.y = CV_MAT_ELEM((*cameraCoordinate), double, 1, 0) * ww;
	point.z = CV_MAT_ELEM((*cameraCoordinate), double, 2, 0) * ww;
	point.w = 1.0;

	return point;
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
	CV_MAT_ELEM((*cameraCoordinate3DPosition), double, 0, 0) = CV_MAT_ELEM((*cameraCoordinatePosition), double, 0, 0);
	CV_MAT_ELEM((*cameraCoordinate3DPosition), double, 1, 0) = CV_MAT_ELEM((*cameraCoordinatePosition), double, 1, 0);
	CV_MAT_ELEM((*cameraCoordinate3DPosition), double, 2, 0) = CV_MAT_ELEM((*cameraCoordinatePosition), double, 2, 0);

	ConvertCamera2Image(output, cameraCoordinate3DPosition);

	CV_MAT_ELEM((*output), double, 0, 0) = CV_MAT_ELEM((*output), double, 0, 0);
	CV_MAT_ELEM((*output), double, 1, 0) = CV_MAT_ELEM((*output), double, 1, 0);
	CV_MAT_ELEM((*output), double, 2, 0) = CV_MAT_ELEM((*output), double, 2, 0);

	cvReleaseMat(&cameraCoordinatePosition);
	cvReleaseMat(&cameraCoordinate3DPosition);

	return 1;
}

windage::Vector2 Calibration::ConvertWorld2Imaged(double x, double y, double z)
{
	CvMat* imageCoordinate = cvCreateMat(3, 1,CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(4, 1,CV_64FC1);

	CV_MAT_ELEM((*worldCoordinate), double, 0, 0) = x;
	CV_MAT_ELEM((*worldCoordinate), double, 1, 0) = y;
	CV_MAT_ELEM((*worldCoordinate), double, 2, 0) = z;
	CV_MAT_ELEM((*worldCoordinate), double, 3, 0) = 1;
	ConvertWorld2Image(imageCoordinate, worldCoordinate);

	double ww = 1.0 / CV_MAT_ELEM((*imageCoordinate), double, 2, 0);
	windage::Vector2 point;
	point.x = CV_MAT_ELEM((*imageCoordinate), double, 0, 0) * ww;
	point.y = CV_MAT_ELEM((*imageCoordinate), double, 1, 0) * ww;

	return point;
}

CvPoint Calibration::ConvertWorld2Image(double x, double y, double z)
{
	CvMat* imageCoordinate = cvCreateMat(3, 1,CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(4, 1,CV_64FC1);

	CV_MAT_ELEM((*worldCoordinate), double, 0, 0) = x;
	CV_MAT_ELEM((*worldCoordinate), double, 1, 0) = y;
	CV_MAT_ELEM((*worldCoordinate), double, 2, 0) = z;
	CV_MAT_ELEM((*worldCoordinate), double, 3, 0) = 1;
	ConvertWorld2Image(imageCoordinate, worldCoordinate);

	double ww = 1.0 / CV_MAT_ELEM((*imageCoordinate), double, 2, 0);
	CvPoint point;
	point.x = cvRound(CV_MAT_ELEM((*imageCoordinate), double, 0, 0) * ww);
	point.y = cvRound(CV_MAT_ELEM((*imageCoordinate), double, 1, 0) * ww);

	cvReleaseMat(&imageCoordinate);
	cvReleaseMat(&worldCoordinate);

	return point;
}


int Calibration::ConvertCamera2World(CvMat* output, CvMat* input)
{
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	CV_MAT_ELEM((*translationVector), double, 0, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 3);
	CV_MAT_ELEM((*translationVector), double, 1, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 3);
	CV_MAT_ELEM((*translationVector), double, 2, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 3);

	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			CV_MAT_ELEM((*rotationMatrix), double, y, x) = CV_MAT_ELEM((*this->extrinsicMatrix), double, y, x);
		}
	}

	CvMat* inversRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvTranspose(rotationMatrix, inversRotationMatrix);

	CvMat* temp = cvCreateMat(3, 1, CV_64FC1);
	cvSub(input, translationVector, temp);
	cvMatMul(inversRotationMatrix, temp, output);

	cvReleaseMat(&temp);

	cvReleaseMat(&inversRotationMatrix);
	cvReleaseMat(&rotationMatrix);
	cvReleaseMat(&translationVector);

	return 1;
}

CvScalar Calibration::ConvertCamera2World(double x, double y, double z)
{
	CvMat* cameraCoordinate = cvCreateMat(3, 1, CV_64FC1);
	CvMat* worldCoordinate = cvCreateMat(3, 1, CV_64FC1);

	CV_MAT_ELEM((*cameraCoordinate), double, 0, 0) = x;
	CV_MAT_ELEM((*cameraCoordinate), double, 1, 0) = y;
	CV_MAT_ELEM((*cameraCoordinate), double, 2, 0) = z;

	ConvertCamera2World(worldCoordinate, cameraCoordinate);

	CvScalar worldPoint;
	worldPoint.val[0] = CV_MAT_ELEM((*worldCoordinate), double, 0, 0);
	worldPoint.val[1] = CV_MAT_ELEM((*worldCoordinate), double, 1, 0);
	worldPoint.val[2] = CV_MAT_ELEM((*worldCoordinate), double, 2, 0);

	cvReleaseMat(&cameraCoordinate);
	cvReleaseMat(&worldCoordinate);

	return worldPoint;
}

int Calibration::ConvertImage2Camera(CvMat* output, CvMat* input, double z)
{
	double rho;

	CvMat* inversIntrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(this->intrinsicMatrix, inversIntrinsicMatrix);

	CvMat* temp = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversIntrinsicMatrix, input, temp);

	rho = z / CV_MAT_ELEM((*temp), double, 2, 0);

	CV_MAT_ELEM((*output), double, 0, 0) = CV_MAT_ELEM((*temp), double, 0, 0) * rho;
	CV_MAT_ELEM((*output), double, 1, 0) = CV_MAT_ELEM((*temp), double, 1, 0) * rho;
	CV_MAT_ELEM((*output), double, 2, 0) = CV_MAT_ELEM((*temp), double, 2, 0) * rho;

	cvReleaseMat(&temp);
	cvReleaseMat(&inversIntrinsicMatrix);

	return 1;
}

int Calibration::ConvertImage2World(CvMat* output, CvMat* input, double z)
{
	CvMat* inversIntrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(this->intrinsicMatrix, inversIntrinsicMatrix);

	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	CV_MAT_ELEM((*translationVector), double, 0, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 0, 3);
	CV_MAT_ELEM((*translationVector), double, 1, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 1, 3);
	CV_MAT_ELEM((*translationVector), double, 2, 0) = CV_MAT_ELEM((*this->extrinsicMatrix), double, 2, 3);

	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			CV_MAT_ELEM((*rotationMatrix), double, x, y) = CV_MAT_ELEM((*this->extrinsicMatrix), double, x, y);
		}
	}

//	CvMat* inversRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
//	cvInvert(rotationMatrix, inversRotationMatrix);

	CvMat* imageDot = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversIntrinsicMatrix, input, imageDot);

	CvMat* tempMatrix = cvCreateMat(3, 3, CV_64FC1);
	CV_MAT_ELEM((*tempMatrix), double, 0, 0) = - CV_MAT_ELEM((*imageDot), double, 0, 0);
	CV_MAT_ELEM((*tempMatrix), double, 0, 1) = CV_MAT_ELEM((*rotationMatrix), double, 0, 0);
	CV_MAT_ELEM((*tempMatrix), double, 0, 2) = CV_MAT_ELEM((*rotationMatrix), double, 0, 1);

	CV_MAT_ELEM((*tempMatrix), double, 1, 0) = - CV_MAT_ELEM((*imageDot), double, 1, 0);
	CV_MAT_ELEM((*tempMatrix), double, 1, 1) = CV_MAT_ELEM((*rotationMatrix), double, 1, 0);
	CV_MAT_ELEM((*tempMatrix), double, 1, 2) = CV_MAT_ELEM((*rotationMatrix), double, 1, 1);

	CV_MAT_ELEM((*tempMatrix), double, 2, 0) = - CV_MAT_ELEM((*imageDot), double, 2, 0);
	CV_MAT_ELEM((*tempMatrix), double, 2, 1) = CV_MAT_ELEM((*rotationMatrix), double, 2, 0);
	CV_MAT_ELEM((*tempMatrix), double, 2, 2) = CV_MAT_ELEM((*rotationMatrix), double, 2, 1);

	CvMat* inversTempMatrix = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(tempMatrix, inversTempMatrix);

	CvMat* tempVector = cvCreateMat(3, 1, CV_64FC1);
	CV_MAT_ELEM((*tempVector), double, 0, 0) = - (CV_MAT_ELEM((*rotationMatrix), double, 0, 2) * z + CV_MAT_ELEM((*translationVector), double, 0, 0));
	CV_MAT_ELEM((*tempVector), double, 1, 0) = - (CV_MAT_ELEM((*rotationMatrix), double, 1, 2) * z + CV_MAT_ELEM((*translationVector), double, 1, 0));
	CV_MAT_ELEM((*tempVector), double, 2, 0) = - (CV_MAT_ELEM((*rotationMatrix), double, 2, 2) * z + CV_MAT_ELEM((*translationVector), double, 2, 0));

	CvMat* tempResult = cvCreateMat(3, 1, CV_64FC1);
	cvMatMul(inversTempMatrix, tempVector, tempResult);

	double rho = CV_MAT_ELEM((*tempResult), double, 0, 0);

	CV_MAT_ELEM((*output), double, 0, 0) = CV_MAT_ELEM((*tempResult), double, 1, 0);
	CV_MAT_ELEM((*output), double, 1, 0) = CV_MAT_ELEM((*tempResult), double, 2, 0);
	CV_MAT_ELEM((*output), double, 2, 0) = z;

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

	CV_MAT_ELEM((*imageCoordinate), double, 0, 0) = ix;
	CV_MAT_ELEM((*imageCoordinate), double, 1, 0) = iy;
	CV_MAT_ELEM((*imageCoordinate), double, 2, 0) = 1;
	ConvertImage2World(worldCoordinate, imageCoordinate, wz);

	CvPoint2D64f point;
	point.x = CV_MAT_ELEM((*worldCoordinate), double, 0, 0);
	point.y = CV_MAT_ELEM((*worldCoordinate), double, 1, 0);

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
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(size, 0.0, 0.0), CV_RGB(0, 0, 0), 5);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, size, 0.0), CV_RGB(0, 0, 0), 5);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, 0.0, size), CV_RGB(0, 0, 0), 5);

	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(size, 0.0, 0.0), CV_RGB(255, 0, 0), 2);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, size, 0.0), CV_RGB(0, 255, 0), 2);
	cvLine(colorImage, this->ConvertWorld2Image(0.0, 0.0, 0.0), this->ConvertWorld2Image(0.0, 0.0, size), CV_RGB(0, 0, 255), 2);
}