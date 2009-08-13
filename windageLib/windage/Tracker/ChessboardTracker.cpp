#include "ChessboardTracker.h"
using namespace windage;

ChessboardTracker::ChessboardTracker()
{
	cameraParameter = NULL;
	chessboardPoints = NULL;
}

ChessboardTracker::~ChessboardTracker()
{
	this->Release();
}

void ChessboardTracker::Release()
{
	if(cameraParameter) delete cameraParameter;
	cameraParameter = NULL;
	if(chessboardPoints) delete chessboardPoints;
	chessboardPoints = NULL;
}

void ChessboardTracker::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, int chessboardWidth, int chessboardHeight, double fieldSize)
{
	this->Release();
	cameraParameter = new Calibration();
	cameraParameter->Initialize(fx, fy, cx, cy, d1, d2, d3, d4);
	this->SetChessboard(chessboardWidth, chessboardHeight, fieldSize);
	
	int pointCount = this->GetPointCount();
	chessboardPoints = new CvPoint2D32f[pointCount];
}

int ChessboardTracker::FindChessBoardCorner(IplImage* inputImage)
{
	int pointCount = this->GetPointCount();
	CvSize size = cvSize(this->chessboardWidth-1, this->chessboardHeight-1);

	bool isGrayImage = false;
	if(inputImage->nChannels == 1)
		isGrayImage = true;

	IplImage* grayImage = inputImage;
	if(isGrayImage == false)
	{
		grayImage = cvCreateImage(cvSize(inputImage->width, inputImage->height), IPL_DEPTH_8U, 1);
		cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);
	}

	int result = cvFindChessBoardCornerGuesses(grayImage, inputImage, NULL, size, chessboardPoints, &pointCount);
	if(result)
	{
		cvFindCornerSubPix(grayImage, chessboardPoints, pointCount, cvSize(5,5), cvSize(-1,-1),
								   cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));
	}

	if(isGrayImage == false)
		cvReleaseImage(&grayImage);

	return result;
}

void ChessboardTracker::DrawChessBoardCorner(IplImage* inputImage)
{
	int pointCount = this->GetPointCount();
	int r = 255;
	int g = 0;
	int b = 0;

	cvCircle(inputImage, cvPoint(chessboardPoints[0].x, chessboardPoints[0].y), 5, CV_RGB(255, 0, 0), CV_FILLED);
	for(int i=1; i<pointCount; i++)
	{
		r = 255 - 255 * (i/(double)pointCount);
		b = 255 * (i/(double)pointCount);

		cvLine(inputImage, cvPoint(chessboardPoints[i-1].x, chessboardPoints[i-1].y), cvPoint(chessboardPoints[i].x, chessboardPoints[i].y), CV_RGB(r, g, b), 2);
		cvCircle(inputImage, cvPoint(chessboardPoints[i].x, chessboardPoints[i].y), 5, CV_RGB(r, g, b), CV_FILLED);
	}
}

bool ChessboardTracker::UpdateExtrinsicParams()
{
	CvMat* rotationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	int pointCount = this->GetPointCount();

	CvMat* pImagePoints = cvCreateMat(pointCount, 2, CV_64FC1);
	CvMat* objectPoints = cvCreateMat(pointCount, 3, CV_64FC1);

	for (int i=0; i < pointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, chessboardPoints[i].x);
		cvSetReal2D(pImagePoints, i, 1, chessboardPoints[i].y);
	}
	for(int y=0; y<chessboardHeight-1; y++)
	{
		for(int x=0; x<chessboardWidth-1; x++)
		{
				cvSetReal2D(objectPoints, (y * (chessboardWidth-1) + x), 0, (fieldSize * y) );
				cvSetReal2D(objectPoints, (y * (chessboardWidth-1) + x), 1, (fieldSize * x) );
				cvSetReal2D(objectPoints, (y * (chessboardWidth-1) + x), 2, 0.0f );
		}
	}

	cvFindExtrinsicCameraParams2(objectPoints, pImagePoints, cameraParameter->GetIntrinsicMatrix(), cameraParameter->GetDistortionCoefficients(), rotationVector, translationVector);
	cameraParameter->ConvertExtrinsicParameter(rotationVector, translationVector);

	// release matrix
	if(pImagePoints)		cvReleaseMat(&pImagePoints);
	if(objectPoints)		cvReleaseMat(&objectPoints);
	if(rotationVector)		cvReleaseMat(&rotationVector);
	if(translationVector)	cvReleaseMat(&translationVector);

	return true;
}

int ChessboardTracker::UpdateCameraPose(IplImage* grayImage)
{
	if(this->FindChessBoardCorner(grayImage) == true)
	{
		this->UpdateExtrinsicParams();
		return 1;
	}
	else
	{
		return 0;
	}
}