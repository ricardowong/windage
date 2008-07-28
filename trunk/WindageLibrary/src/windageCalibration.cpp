
#include <cv.h>
#include <vector>

#include "../include/windageMatrix.h"

extern "C" __declspec(dllexport)
int FindChessBoardCorner(CvPoint2D32f* resultPoint, IplImage* image, int chessBoardWidth, int chessBoardHeight)
{
	int result;
	int pointCount = (chessBoardWidth-1) * (chessBoardHeight-1);
	CvSize size = cvSize(chessBoardWidth-1, chessBoardHeight-1);
	
	IplImage* grayImage = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image, grayImage, CV_BGR2GRAY);
	
	result = cvFindChessBoardCornerGuesses(grayImage, image, NULL, size, resultPoint, &pointCount);

	if(result)
	{
		//find subpixel corners coordinates
	    cvFindCornerSubPix(grayImage, resultPoint, pointCount, cvSize(5,5), cvSize(-1,-1),
			               cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));
	}

	cvReleaseImage(&grayImage);

	return result;
}

extern "C" __declspec(dllexport)
void SolveCalibration(Matrix3* intrinsicMatrix, Vector4* distortionCoefficients, Vector3* rotationVector, Vector3* translationVector,
					  CvPoint2D32f* cornersArray, int chessBoardWidth, int chessBoardHeight, double fieldSize,
					  int imageWidth, int imageHeight, int imageCount)
{
	int pointCount = (chessBoardWidth-1) * (chessBoardHeight-1);
	int allPointCount = pointCount * imageCount;

	// init object points array
	int i, x, y;

	CvMat* pImagePoints = cvCreateMat(allPointCount, 2, CV_64FC1);
	CvMat* objectPoints = cvCreateMat(allPointCount, 3, CV_64FC1);
	CvMat* imagePointCounts = cvCreateMat(imageCount, 1, CV_32SC1);

	for (int i=0; i < allPointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, cornersArray[i].x);
		cvSetReal2D(pImagePoints, i, 1, cornersArray[i].y);
	}

	for(y=0; y<chessBoardHeight-1; y++)
	{
		for(x=0; x<chessBoardWidth-1; x++)
		{
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 0, (fieldSize * y) );
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 1, (fieldSize * x) );
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 2, 0.0f );
		}
	}
	for(i=0; i<imageCount; i++)
	{
		for(x=0; x<pointCount; x++)
		{
			cvSetReal2D(objectPoints, (i*pointCount + x), 0, cvGetReal2D(objectPoints, x, 0) );
			cvSetReal2D(objectPoints, (i*pointCount + x), 1, cvGetReal2D(objectPoints, x, 1) );
			cvSetReal2D(objectPoints, (i*pointCount + x), 2, 0.0f );
		}
	}

	for(i=0; i<imageCount; i++)
	{
		cvSetReal1D(imagePointCounts, i, pointCount);
	}

	CvMat* cameraMatrix = cvCreateMat(3, 3, CV_64FC1);
	CvMat* distortionCoeffs = cvCreateMat(4, 1, CV_64FC1);
	CvMat* rotationVects = cvCreateMat(imageCount, 3, CV_64FC1);
	CvMat* transVects = cvCreateMat(imageCount, 3, CV_64FC1);

	// calculate calibration function
	cvCalibrateCamera2(objectPoints, pImagePoints, imagePointCounts, cvSize(imageWidth, imageHeight),
						cameraMatrix, distortionCoeffs, rotationVects, transVects, 0);

	// calibration result
	intrinsicMatrix->_11 = cvGetReal2D(cameraMatrix, 0, 0);
	intrinsicMatrix->_12 = 0.0f;
	intrinsicMatrix->_13 = cvGetReal2D(cameraMatrix, 0, 2);
	intrinsicMatrix->_21 = 0.0f;
	intrinsicMatrix->_22 = cvGetReal2D(cameraMatrix, 1, 1);
	intrinsicMatrix->_23 = cvGetReal2D(cameraMatrix, 1, 2);
	intrinsicMatrix->_31 = 0.0f;
	intrinsicMatrix->_32 = 0.0f;
	intrinsicMatrix->_33 = 1.0f;

	for(i=0; i<4; i++)
	{
		distortionCoefficients->v[i] = cvGetReal1D(distortionCoeffs, i);
	}

	for(i=0; i<imageCount; i++)
	{
		for(x=0; x<3; x++)
		{
			translationVector[i].v[x] = cvGetReal2D(transVects, i, x);
			rotationVector[i].v[x] = cvGetReal2D(rotationVects, i, x);
		}
	}

	// release matrix
	if(pImagePoints != NULL)		cvReleaseMat(&pImagePoints);
	if(objectPoints != NULL)		cvReleaseMat(&objectPoints);
	if(imagePointCounts != NULL)	cvReleaseMat(&imagePointCounts);

	if(cameraMatrix != NULL)		cvReleaseMat(&cameraMatrix);
	if(distortionCoeffs != NULL)	cvReleaseMat(&distortionCoeffs);
	if(rotationVects != NULL)		cvReleaseMat(&rotationVects);
	if(transVects != NULL)			cvReleaseMat(&transVects);
}

extern "C" __declspec(dllexport)
void UpdateExtrinsicParams(Vector3* rotationVector, Vector3* translationVector,
					  Matrix3 intrinsicMatrix, Vector4 distortionCoefficients,
					  CvPoint2D32f* cornersArray, int chessBoardWidth, int chessBoardHeight, double fieldSize,
					  int imageWidth, int imageHeight)
{
	int x, y, i;
	int pointCount = (chessBoardWidth-1) * (chessBoardHeight-1);

	CvMat* pImagePoints = cvCreateMat(pointCount, 2, CV_64FC1);
	CvMat* objectPoints = cvCreateMat(pointCount, 3, CV_64FC1);

	for (i=0; i < pointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, cornersArray[i].x);
		cvSetReal2D(pImagePoints, i, 1, cornersArray[i].y);
	}

	for(y=0; y<chessBoardHeight-1; y++)
	{
		for(x=0; x<chessBoardWidth-1; x++)
		{
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 0, (fieldSize * y) );
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 1, (fieldSize * x) );
			cvSetReal2D(objectPoints, (y * (chessBoardWidth-1) + x), 2, 0.0f );
		}
	}

	CvMat* cameraMatrix = cvCreateMat(3, 3, CV_64FC1);
	CvMat* distortionCoeffs = cvCreateMat(4, 1, CV_64FC1);
	CvMat* rotationVects = cvCreateMat(1, 3, CV_64FC1);
	CvMat* transVects = cvCreateMat(1, 3, CV_64FC1);

	for(y=0; y<3; y++)
	{
		for(x=0; x<3; x++)
		{
			cvSetReal2D(cameraMatrix, y, x, intrinsicMatrix.m[y][x]);
		}
	}
	for(i=0; i<4; i++)
	{
		cvSetReal1D(distortionCoeffs, i, distortionCoefficients.v[i]);
	}

	cvFindExtrinsicCameraParams2(objectPoints, pImagePoints, cameraMatrix, distortionCoeffs, rotationVects, transVects);

	// calibration update result
	for(x=0; x<3; x++)
	{
		translationVector->v[x] = cvGetReal2D(transVects, 0, x);
		rotationVector->v[x] = cvGetReal2D(rotationVects, 0, x);
	}

	// release matrix
	if(pImagePoints != NULL)		cvReleaseMat(&pImagePoints);
	if(objectPoints != NULL)		cvReleaseMat(&objectPoints);

	if(cameraMatrix != NULL)		cvReleaseMat(&cameraMatrix);
	if(distortionCoeffs != NULL)	cvReleaseMat(&distortionCoeffs);
	if(rotationVects != NULL)		cvReleaseMat(&rotationVects);
	if(transVects != NULL)			cvReleaseMat(&transVects);
}

extern "C" __declspec(dllexport)
void GetExtrinsicMatrix(Matrix4* extrinsicMatrix, Vector3 rotationVector, Vector3 translationVector)
{
	CvMat *rotation_vect=NULL;
	CvMat* rotation_matrix=NULL;
	CvMat* ExtrinsicMat=NULL;
	CvMat* ExtrinsicMatInv=NULL;
	
	rotation_vect = cvCreateMat(3, 1, CV_64FC1);
	rotation_matrix = cvCreateMat(3, 3, CV_64FC1);
	
	ExtrinsicMat = cvCreateMat(4, 4, CV_64FC1);
	ExtrinsicMatInv = cvCreateMat(4, 4, CV_64FC1);

	cvSetZero(ExtrinsicMat);
	int i, j;

	for(j=0; j < 3; j++) 
	{
		cvSetReal1D(rotation_vect, j, rotationVector.v[j]);
		//set trans vect. in extrinsic matrix
		cvSetReal2D(ExtrinsicMat, j, 3, translationVector.v[j]);
	} 
	cvRodrigues2(rotation_vect, rotation_matrix);

	for(i=0; i < 3; i++) 
	{
		for(j=0; j < 3; j++) 
		{
			//set rot. vect. in extrinsic matrix
			cvSetReal2D(ExtrinsicMat, i, j, cvGetReal2D(rotation_matrix, i, j));      
		}
	}

	//set last element of the extrinsic matrix
	cvSetReal2D(ExtrinsicMat, 3, 3, 1.0); 

	cvInvert(ExtrinsicMat, ExtrinsicMatInv);

	//copy invert matrix
	for (int i=0; i < 4; ++i) 
	{
		for (int j=0; j < 4; ++j) 
		{
			extrinsicMatrix->m[i][j] = cvGetReal2D(ExtrinsicMatInv, i, j);
		}
	}
	
	cvReleaseMat( &rotation_vect );
	cvReleaseMat( &rotation_matrix );
	cvReleaseMat( &ExtrinsicMat );
	cvReleaseMat( &ExtrinsicMatInv );
}

extern "C" __declspec(dllexport)
void UnRadialDistortion(IplImage* result, IplImage* input, Matrix3 instrinsicMatrix, Vector4 distortionCoefficients)
{
	CvMat* cameraMatrix = cvCreateMat(3, 3, CV_64FC1);
	CvMat* distortionCoeffis = cvCreateMat(4, 1, CV_64FC1);

	int x, y;
	for(y=0; y<3; y++)
	{
		for(x=0; x<3; x++)
		{
			cvSetReal2D(cameraMatrix, y, x, instrinsicMatrix.m[y][x]);
		}

		cvSetReal1D(distortionCoeffis, y, distortionCoefficients.v[y]);
	}
	
	cvUndistort2(input, result, cameraMatrix, distortionCoeffis);

	cvReleaseMat(&cameraMatrix);
	cvReleaseMat(&distortionCoeffis);
}
