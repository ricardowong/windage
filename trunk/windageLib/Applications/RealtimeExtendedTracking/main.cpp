#ifdef RUNNING

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

#include <iostream>

#include <highgui.h>
#include <windage.h>

#include "PlaneEstimation.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const char* templateFileName = "D:\\ImageSequence\\20090828\\capture%d.jpg";

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	windage::ChessboardTracker* chessboardTracker = new windage::ChessboardTracker();
	chessboardTracker->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 7, 8, 28.0);
	chessboardTracker->SetUsingSubpixelAccuracy(true);

	windage::ChessboardTracker* savedTracker = new windage::ChessboardTracker();
	savedTracker->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 7, 8, 28.0);
	savedTracker->SetUsingSubpixelAccuracy(true);

	windage::OpticalFlow* opticalFlow = new windage::OpticalFlow();
	opticalFlow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
//	opticalFlow->SetRemovePrevPoints(true);

	char filename[100];
	sprintf(filename, templateFileName, 0);
	IplImage* tempImage = cvLoadImage(filename);
	IplImage* inputImage = cvCreateImage(cvGetSize(tempImage), IPL_DEPTH_8U, 3);
	IplImage* savedImage = cvCreateImage(cvGetSize(tempImage), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(tempImage), IPL_DEPTH_8U, 1);
	IplImage* prevImage = cvCreateImage(cvGetSize(tempImage), IPL_DEPTH_8U, 1);
	cvReleaseImage(&tempImage);

	std::vector<CvPoint> cornerPoints;
	std::vector<CvPoint2D32f> prevPoints;
	std::vector<CvPoint2D32f> currPoints;

	std::vector<windage::Vector3> worldPoints;
	std::vector<windage::Vector3> planeConsensus;

	cvNamedWindow("saved image");
	cvNamedWindow("result image");

	bool isReconstruction = false;
	bool isTracking = true;
	bool isExtractFastFeature = false;
	bool isOpticalFlow = false;
	bool isBreak = false;
	bool isProcessing = true;

	const int roopStep = 1;
	for(int is=1000; is<2000&&isProcessing; is+=roopStep)
	{
		fpslog->log("fps", fpslog->calculateFPS());
		fpslog->logNewLine();
		fpslog->updateTickCount();

		sprintf(filename, templateFileName, is);
		tempImage = cvLoadImage(filename);
		calibration->Undistortion(tempImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);

		cvReleaseImage(&tempImage);

		if(isTracking)
		{
			chessboardTracker->UpdateCameraPose(grayImage);
			chessboardTracker->DrawInfomation(inputImage, 100.0);
		}

		if(isOpticalFlow)
		{
			opticalFlow->TrackFeature(prevImage, grayImage, &prevPoints, &currPoints);

			for(int i=0; i<currPoints.size(); i++)
			{
				cvCircle(inputImage, cvPoint(cornerPoints[i].x, cornerPoints[i].y), 2, CV_RGB(0, 255, 0), CV_FILLED);
				cvCircle(inputImage, cvPoint(currPoints[i].x, currPoints[i].y), 2, CV_RGB(255, 0, 0), CV_FILLED);
				cvLine(inputImage, cvPoint(cornerPoints[i].x, cornerPoints[i].y), cvPoint(currPoints[i].x, currPoints[i].y), CV_RGB(255, 255, 0));
			}
			
			prevPoints.clear();
			for(int i=0; i<currPoints.size(); i++)
				prevPoints.push_back(currPoints[i]);
			currPoints.clear();
		}

		if(isExtractFastFeature)
		{
			cornerPoints.clear();
			const int FAST_THRESHOLD = 60;
			windage::ModifiedSURFTracker::ExtractFASTCorner(&cornerPoints, grayImage, FAST_THRESHOLD);

			// push opticalflow points
			prevPoints.clear();
			for(int i=0; i<cornerPoints.size(); i++)
				prevPoints.push_back(cvPoint2D32f((double)cornerPoints[i].x, (double)cornerPoints[i].y));

			// save camera pose
			calibration->SetExtrinsicMatrix(chessboardTracker->GetCameraParameter()->GetExtrinsicMatrix());
			savedTracker->GetCameraParameter()->SetExtrinsicMatrix(chessboardTracker->GetCameraParameter()->GetExtrinsicMatrix());

			cvCopyImage(inputImage, savedImage);

			isOpticalFlow = true;
			isExtractFastFeature = false;
		}

		if(isReconstruction)
		{
			worldPoints.clear();
			for(int i=0; i<prevPoints.size(); i++)
			{
				if(prevPoints[i].x > 0.0)
				{
					CvPoint point1 = cornerPoints[i];
					CvPoint point2 = cvPoint(prevPoints[i].x, prevPoints[i].y);

					CvScalar worldPoint = windage::Reconstructor::Calc3DPointApproximation(calibration, chessboardTracker->GetCameraParameter(), point1, point2);

					windage::Vector3 worldVector = windage::Vector3(worldPoint.val[0], worldPoint.val[1], worldPoint.val[2]);

					if( -28.0 <= worldVector.x && worldVector.x < 230.0 &&
						-28.0 <= worldVector.y && worldVector.y < 190.0)
					{
					}
					else
						worldPoints.push_back(worldVector);
				}
			}

			for(int i=0; i<worldPoints.size(); i++)
			{
				const int size = 3;
				CvPoint point1 = chessboardTracker->GetCameraParameter()->ConvertWorld2Image(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
				cvRectangle(inputImage, cvPoint(point1.x - size, point1.y - size), cvPoint(point1.x + size, point1.y + size), CV_RGB(0, 255, 0));

				CvPoint point2 = calibration->ConvertWorld2Image(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
				cvRectangle(savedImage, cvPoint(point2.x - size, point2.y - size), cvPoint(point2.x + size, point2.y + size), CV_RGB(0, 255, 0));
			}

			Vector3 center;
			Vector4 plane = PlaneEstimationRANSAC(&worldPoints, &planeConsensus, center);

			for(int i=0; i<planeConsensus.size(); i++)
			{
				const int size = 2;
				CvPoint point1 = chessboardTracker->GetCameraParameter()->ConvertWorld2Image(planeConsensus[i].x, planeConsensus[i].y, planeConsensus[i].z);
				cvRectangle(inputImage, cvPoint(point1.x - size, point1.y - size), cvPoint(point1.x + size, point1.y + size), CV_RGB(255, 0, 0), CV_FILLED);

				CvPoint point2 = calibration->ConvertWorld2Image(planeConsensus[i].x, planeConsensus[i].y, planeConsensus[i].z);
				cvRectangle(savedImage, cvPoint(point2.x - size, point2.y - size), cvPoint(point2.x + size, point2.y + size), CV_RGB(255, 0, 0), CV_FILLED);
			}

			// calcuate extrinsic parameter
			double extrinsicData[16];
			CvMat extrinsic = cvMat(4, 4, CV_64FC1, extrinsicData);

			std::vector<Vector2> imagePoints;
			std::vector<Vector3> objectPoints;
			Vector3 normal= Vector3(plane.x, plane.y, plane.z);
			normal /= normal.getLength();

			CvPoint imagePoint;
			for(int i=0; i<planeConsensus.size(); i++)
			{
				imagePoint = calibration->ConvertWorld2Image(planeConsensus[i].x, planeConsensus[i].y, planeConsensus[i].z);
				imagePoints.push_back(Vector2(imagePoint.x, imagePoint.y));

				Vector3 temp = ConvertWorld2PlaneCoordinate(planeConsensus[i]-center, normal);
				objectPoints.push_back(temp);
			}

			CalculatePose(calibration->GetIntrinsicMatrix(), calibration->GetDistortionCoefficients(), &extrinsic, &imagePoints, &objectPoints);
			savedTracker->GetCameraParameter()->SetExtrinsicMatrix(&extrinsic);
			savedTracker->DrawInfomation(savedImage, 100.0);
//*
			// rectification
			const int SIZE = 400;
			const double SCALE = 0.5;

			IplImage* rectifiedImage = cvCreateImage(cvSize(SIZE, SIZE), IPL_DEPTH_8U, 3);
			for(int y=-SIZE/2; y<SIZE/2; y++)
			{
				for(int x=-SIZE/2; x<SIZE/2; x++)
				{
					double dx = (double)x * SCALE;
					double dy = (double)y * -SCALE;
					double dz = 0.0;

					CvPoint projectedPoint;
					projectedPoint = savedTracker->GetCameraParameter()->ConvertWorld2Image(dx, dy, dz);
					if( 0 <= projectedPoint.x && projectedPoint.x < savedImage->width &&
						0 <= projectedPoint.y && projectedPoint.y < savedImage->height)
					{
						cvSet2D(rectifiedImage, y+SIZE/2, x+SIZE/2, cvGet2D(savedImage, projectedPoint.y, projectedPoint.x));
					}
					else
					{
						cvSet2D(rectifiedImage, y+SIZE/2, x+SIZE/2, cvScalarAll(0));
					}
				}
			}

			cvNamedWindow("rectified");
			cvShowImage("rectified", rectifiedImage);
			cvSaveImage("rectified.png", rectifiedImage);
			cvReleaseImage(&rectifiedImage);
//*/
			isReconstruction = false;
		}

		cvCopyImage(grayImage, prevImage);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), filename);
		cvShowImage("result image", inputImage);
		cvShowImage("saved image", savedImage);

		char ch;
		if(isBreak) ch = cvWaitKey();
		else		ch = cvWaitKey(1);
		switch(ch)
		{
		case 'r':
		case 'R':
			isReconstruction = true;
			break;
		case 't':
		case 'T':
			isTracking = !isTracking;
			break;
		case 'f':
		case 'F':
			isExtractFastFeature = true;
			break;
		case 'o':
		case 'O':
			isOpticalFlow = false;
			break;
		case 's':
		case 'S':
			if(inputImage) cvSaveImage("inputImage.png", inputImage);
			if(savedImage) cvSaveImage("savedImage.png", savedImage);
			break;
		case ' ':
			isBreak = !isBreak;
			break;
		case 'q':
		case 'Q':
			isProcessing = false;
			break;
		}
	}

	cvDestroyAllWindows();
}

#endif