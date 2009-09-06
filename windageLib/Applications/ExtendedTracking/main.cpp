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

#include "KDTreeWrapper.h"
#include "PlaneEstimation.h"

#define RECTIFICATION

const int WIDTH = 640;
const int HEIGHT = 480;

windage::Tracker* tracker1;
windage::Tracker* tracker2;
windage::Tracker* updateTracker1;
windage::Tracker* updateTracker2;

Vector3 center;
Vector4 plane;

typedef struct _featurePoints
{
	Vector3 point;
	bool isSelected;
	void operator=(const struct _featurePoints rhs)
	{
		this->point = rhs.point;
		this->isSelected = rhs.isSelected;
	}
}featurePoints;

std::vector<featurePoints> worldFeaturePoints;
std::vector<featurePoints> worldSelectedFeaturePoints;
std::vector<Vector3> consensus;

bool isLeftDown = false;
bool isRightDown = false;
void MouseEvent( int mevent, int x, int y, int flags, void* param )
{
   switch(mevent)
   {
   case CV_EVENT_LBUTTONDOWN:
	   isLeftDown = true;
	   break;
   case CV_EVENT_LBUTTONUP:
	   isLeftDown = false;
	   break;
   case CV_EVENT_RBUTTONDOWN:
	   isRightDown = true;
	   break;
   case CV_EVENT_RBUTTONUP:
	   isRightDown = false;
	   break;
   }

   if(isLeftDown || isRightDown)
   {
	   Vector2 mousePoint = Vector2(x, y);

	   for(int i=0; i<worldFeaturePoints.size(); i++)
	   {
		   if(x < 640)
		   {
			   CvPoint imagePoint = tracker1->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
			   Vector2 imagePointVector = Vector2(imagePoint.x, imagePoint.y);

			   if(mousePoint.getDistance(imagePointVector) < 10)
			   {
				   if(isLeftDown)
					   worldFeaturePoints[i].isSelected = true;
				   if(isRightDown)
					   worldFeaturePoints[i].isSelected = false;
			   }
		   }
		   else
		   {
			   CvPoint imagePoint = tracker2->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
			   Vector2 imagePointVector = Vector2(imagePoint.x + 640, imagePoint.y);

			   if(mousePoint.getDistance(imagePointVector) < 10)
			   {
				   if(isLeftDown)
					   worldFeaturePoints[i].isSelected = true;
				   if(isRightDown)
					   worldFeaturePoints[i].isSelected = false;
			   }
		   }
	   }
   }
}

void main()
{
	Logger* log = new Logger(&std::cout);

	// Tracker Initialize
	IplImage* referenceImage = cvLoadImage("reference.png", 0);

	tracker1 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 20);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 6);
	((windage::ModifiedSURFTracker*)tracker1)->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 20);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 6);
	((windage::ModifiedSURFTracker*)tracker2)->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	updateTracker1 = new windage::ModifiedSURFTracker();
	updateTracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)updateTracker1)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 20);
	((windage::ModifiedSURFTracker*)updateTracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 20);

	IplImage* temp1 = cvLoadImage("extended/input1.jpg");
	IplImage* temp2 = cvLoadImage("extended/input2.jpg");
	IplImage* input1 = cvCreateImage(cvGetSize(temp1), IPL_DEPTH_8U, 3);
	IplImage* input2 = cvCreateImage(cvGetSize(temp2), IPL_DEPTH_8U, 3);
	IplImage* gray1 = cvCreateImage(cvGetSize(temp1), IPL_DEPTH_8U, 1);
	IplImage* gray2 = cvCreateImage(cvGetSize(temp2), IPL_DEPTH_8U, 1);

#ifdef RECTIFICATION
	tracker1->GetCameraParameter()->Undistortion(temp1, input1);
	tracker2->GetCameraParameter()->Undistortion(temp2, input2);
#else
	cvCopy(temp1, input1);
	cvCopy(temp2, input2);
#endif
	cvCvtColor(input1, gray1, CV_BGR2GRAY);
	cvCvtColor(input2, gray2, CV_BGR2GRAY);

	IplImage* resultTempImage = cvCreateImage(cvSize(input1->width*2, input1->height), IPL_DEPTH_8U, 3);
	IplImage* resultImage = cvCreateImage(cvSize(input1->width*2, input1->height), IPL_DEPTH_8U, 3);

	cvNamedWindow("result");
	cvSetMouseCallback("result",MouseEvent);

	log->updateTickCount();
	tracker1->UpdateCameraPose(gray1);
	tracker2->UpdateCameraPose(gray2);
	log->log("TRACK", log->calculateProcessTime());

	tracker1->DrawInfomation(input1);
	tracker2->DrawInfomation(input2);

//	tracker1->DrawDebugInfo(input1);
//	tracker2->DrawDebugInfo(input2);

	std::vector<CvPoint> points1;
	std::vector<CvPoint> points2;

	log->updateTickCount();
	windage::ModifiedSURFTracker::ExtractFASTCorner(&points1, gray1, 30);
	windage::ModifiedSURFTracker::ExtractFASTCorner(&points2, gray2, 30);
	log->log("FAST", log->calculateProcessTime());
/*
	for(int i=0; i<points1.size(); i++)
		cvCircle(input1, points1[i], 5, CV_RGB(255, 0, 0));
	for(int i=0; i<points2.size(); i++)
		cvCircle(input2, points2[i], 5, CV_RGB(255, 0, 0));
//*/
	std::vector<SURFDesciription> descriptions1;
	std::vector<SURFDesciription> descriptions2;

	log->updateTickCount();
	windage::ModifiedSURFTracker::ExtractModifiedSURF(gray1, &points1, &descriptions1);
	windage::ModifiedSURFTracker::ExtractModifiedSURF(gray2, &points2, &descriptions2);
	log->log("SURF", log->calculateProcessTime());

	cvSetImageROI(resultTempImage, cvRect(0, 0, input1->width, input1->height));
	cvCopy(input1, resultTempImage);
	cvSetImageROI(resultTempImage, cvRect(input1->width, 0, input1->width, input1->height));
	cvCopy(input2, resultTempImage);
	cvResetImageROI(resultTempImage);

	int count = (int)descriptions1.size();
	log->updateTickCount();
	CvMat* referenceFeatureStorage = cvCreateMat(count, DESCRIPTOR_DIMENSION, CV_32FC1);
	CvFeatureTree* tree = CreateReferenceTree(&descriptions1, referenceFeatureStorage);
	log->log("TREE", log->calculateProcessTime());

	std::vector<SURFDesciription> matchecPoint1;
	std::vector<SURFDesciription> matchecPoint2;

	log->updateTickCount();
	for(int i=0; i<descriptions2.size(); i++)
	{
		int index = FindPairs(descriptions2[i], tree, 0.7);
		if(index >= 0)
		{

			matchecPoint1.push_back(descriptions1[index]);
			matchecPoint2.push_back(descriptions2[i]);
		}
	}
	log->log("MATCH", log->calculateProcessTime());
	log->logNewLine();

	for(int i=0; i<matchecPoint1.size(); i++)
	{
		CvScalar world = windage::Reconstructor::Calc3DPointApproximation(tracker1->GetCameraParameter(), tracker2->GetCameraParameter(),
			cvPoint(matchecPoint1[i].point.x, matchecPoint1[i].point.y), cvPoint(matchecPoint2[i].point.x, matchecPoint2[i].point.y));

		featurePoints point;
		point.point = Vector3(world.val[0], world.val[1], world.val[2]);;
		point.isSelected = false;
		worldFeaturePoints.push_back(point);
	}

	bool isProcessing = true;
	while(isProcessing)
	{
		cvCopy(resultTempImage, resultImage);

		for(int i=0; i<worldFeaturePoints.size(); i++)
		{
			if(worldFeaturePoints[i].isSelected)
			{
				cvSetImageROI(resultImage, cvRect(0, 0, input1->width, input1->height));
				CvPoint imagePoint = tracker1->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
				cvRectangle(resultImage, cvPoint(imagePoint.x - 3, imagePoint.y - 3), cvPoint(imagePoint.x + 3, imagePoint.y + 3), CV_RGB(255, 0, 0), CV_FILLED);

				cvSetImageROI(resultImage, cvRect(input1->width, 0, input1->width, input1->height));
				imagePoint = tracker2->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
				cvRectangle(resultImage, cvPoint(imagePoint.x - 3, imagePoint.y - 3), cvPoint(imagePoint.x + 3, imagePoint.y + 3), CV_RGB(255, 0, 0), CV_FILLED);
			}
			else
			{
				cvSetImageROI(resultImage, cvRect(0, 0, input1->width, input1->height));
				CvPoint imagePoint = tracker1->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
				cvRectangle(resultImage, cvPoint(imagePoint.x - 3, imagePoint.y - 3), cvPoint(imagePoint.x + 3, imagePoint.y + 3), CV_RGB(0, 255, 0));

				cvSetImageROI(resultImage, cvRect(input1->width, 0, input1->width, input1->height));
				imagePoint = tracker2->GetCameraParameter()->ConvertWorld2Image(worldFeaturePoints[i].point.x, worldFeaturePoints[i].point.y, worldFeaturePoints[i].point.z);
				cvRectangle(resultImage, cvPoint(imagePoint.x - 3, imagePoint.y - 3), cvPoint(imagePoint.x + 3, imagePoint.y + 3), CV_RGB(0, 255, 0));
			}
		}

		// draw consensus set
		for(int i=0; i<consensus.size(); i++)
		{
			cvSetImageROI(resultImage, cvRect(0, 0, input1->width, input1->height));
			CvPoint imagePoint = tracker1->GetCameraParameter()->ConvertWorld2Image(consensus[i].x, consensus[i].y, consensus[i].z);
			cvRectangle(resultImage, cvPoint(imagePoint.x - 2, imagePoint.y - 2), cvPoint(imagePoint.x + 2, imagePoint.y + 2), CV_RGB(255, 255, 0), CV_FILLED);

			cvSetImageROI(resultImage, cvRect(input1->width, 0, input1->width, input1->height));
			imagePoint = tracker2->GetCameraParameter()->ConvertWorld2Image(consensus[i].x, consensus[i].y, consensus[i].z);
			cvRectangle(resultImage, cvPoint(imagePoint.x - 2, imagePoint.y - 2), cvPoint(imagePoint.x + 2, imagePoint.y + 2), CV_RGB(255, 255, 0), CV_FILLED);
		}
/*
		// calculate reference
		CvPoint normal1;
		CvPoint normal2;

		Vector3 normalVector = Vector3(plane.x, plane.y, plane.z);
		normalVector /= normalVector.getLength();
		normalVector *= 10.0;
		normalVector += center;

		// draw normal vector
		cvSetImageROI(resultImage, cvRect(0, 0, input1->width, input1->height));
		normal1 = tracker1->GetCameraParameter()->ConvertWorld2Image(center.x, center.y, center.z);
		normal2 = tracker1->GetCameraParameter()->ConvertWorld2Image(normalVector.x, normalVector.y, normalVector.z);
		cvLine(resultImage, normal1, normal2, CV_RGB(0, 0, 255), 3);
		cvCircle(resultImage, normal1, 3, CV_RGB(255, 0, 0), CV_FILLED);
		cvCircle(resultImage, normal2, 3, CV_RGB(0, 255, 0), CV_FILLED);

		cvSetImageROI(resultImage, cvRect(input1->width, 0, input1->width, input1->height));
		normal1 = tracker2->GetCameraParameter()->ConvertWorld2Image(center.x, center.y, center.z);
		normal2 = tracker2->GetCameraParameter()->ConvertWorld2Image(normalVector.x, normalVector.y, normalVector.z);
		cvLine(resultImage, normal1, normal2, CV_RGB(0, 0, 255), 3);
		cvCircle(resultImage, normal1, 3, CV_RGB(255, 0, 0), CV_FILLED);
		cvCircle(resultImage, normal2, 3, CV_RGB(0, 255, 0), CV_FILLED);
*/

		cvSetImageROI(resultImage, cvRect(0, 0, input1->width, input1->height));
		updateTracker1->DrawInfomation(resultImage);
		cvSetImageROI(resultImage, cvRect(input1->width, 0, input1->width, input1->height));
		updateTracker2->DrawInfomation(resultImage);


		cvResetImageROI(resultImage);
		cvShowImage("result", resultImage);
		
		char ch = cvWaitKey(10);
		switch(ch)
		{
		case 'q':
		case 'Q':
			isProcessing = false;
			break;
		case 'p':
		case 'P':
			worldSelectedFeaturePoints.clear();
			for(int i=0; i<worldFeaturePoints.size(); i++)
			{
				if(worldFeaturePoints[i].isSelected)
				{
					worldSelectedFeaturePoints.push_back(worldFeaturePoints[i]);
				}
			}
			break;
		case 'e':
		case 'E':
			{
				worldSelectedFeaturePoints.clear();
				for(int i=0; i<worldFeaturePoints.size(); i++)
				{
					if(worldFeaturePoints[i].isSelected)
					{
						worldSelectedFeaturePoints.push_back(worldFeaturePoints[i]);
					}
				}

				consensus.clear();
				std::vector<Vector3> points;
				for(int i=0; i<worldSelectedFeaturePoints.size(); i++)
				{
					points.push_back(worldSelectedFeaturePoints[i].point);
				}
				
				plane = PlaneEstimationRANSAC(&points, &consensus, center);
			}
			break;
		case 'd':
		case 'D':
			{
				double extrinsicData[16];
				CvMat extrinsic = cvMat(4, 4, CV_64FC1, extrinsicData);

				std::vector<Vector2> imagePoints1;
				std::vector<Vector2> imagePoints2;
				std::vector<Vector3> objectPoints;
				Vector3 normal= Vector3(plane.x, plane.y, plane.z);
				normal /= normal.getLength();

				CvPoint imagePoint;
				for(int i=0; i<consensus.size(); i++)
				{
					imagePoint = tracker1->GetCameraParameter()->ConvertWorld2Image(consensus[i].x, consensus[i].y, consensus[i].z);
					imagePoints1.push_back(Vector2(imagePoint.x, imagePoint.y));
					imagePoint = tracker2->GetCameraParameter()->ConvertWorld2Image(consensus[i].x, consensus[i].y, consensus[i].z);
					imagePoints2.push_back(Vector2(imagePoint.x, imagePoint.y));

					Vector3 temp = ConvertWorld2PlaneCoordinate(consensus[i]-center, normal);
//					temp = ConvertWorld2PlaneCoordinate(temp, Vector3(0, 0, 1), -0.3);
					
					objectPoints.push_back(temp);
				}

				CalculatePose(tracker1->GetCameraParameter()->GetIntrinsicMatrix(), tracker1->GetCameraParameter()->GetDistortionCoefficients(), &extrinsic, &imagePoints1, &objectPoints);
				updateTracker1->GetCameraParameter()->SetExtrinsicMatrix(&extrinsic);

				CalculatePose(tracker2->GetCameraParameter()->GetIntrinsicMatrix(), tracker1->GetCameraParameter()->GetDistortionCoefficients(), &extrinsic, &imagePoints2, &objectPoints);
				updateTracker2->GetCameraParameter()->SetExtrinsicMatrix(&extrinsic);
			}
			break;
		case 'r':
		case 'R':
			{
				const int SIZE = 400;
				const double SCALE = 0.05;

				IplImage* rectifiedImage1 = cvCreateImage(cvSize(SIZE, SIZE), IPL_DEPTH_8U, 3);
				IplImage* rectifiedImage2 = cvCreateImage(cvSize(SIZE, SIZE), IPL_DEPTH_8U, 3);

				for(int y=-SIZE/2; y<SIZE/2; y++)
				{
					for(int x=-SIZE/2; x<SIZE/2; x++)
					{
						double dx = (double)x * SCALE;
						double dy = (double)y * -SCALE;
						double dz = 0.0;

						CvPoint projectedPoint;
						projectedPoint = updateTracker1->GetCameraParameter()->ConvertWorld2Image(dx, dy, dz);
						if( 0 <= projectedPoint.x && projectedPoint.x < input1->width &&
							0 <= projectedPoint.y && projectedPoint.y < input1->height)
						{
							cvSet2D(rectifiedImage1, y+SIZE/2, x+SIZE/2, cvGet2D(input1, projectedPoint.y, projectedPoint.x));
						}
						else
						{
							cvSet2D(rectifiedImage1, y+SIZE/2, x+SIZE/2, cvScalarAll(0));
						}

						projectedPoint = updateTracker2->GetCameraParameter()->ConvertWorld2Image(dx, dy, dz);
						if( 0 <= projectedPoint.x && projectedPoint.x < input1->width &&
							0 <= projectedPoint.y && projectedPoint.y < input1->height)
						{
							cvSet2D(rectifiedImage2, y+SIZE/2, x+SIZE/2, cvGet2D(input2, projectedPoint.y, projectedPoint.x));
						}
						else
						{
							cvSet2D(rectifiedImage2, y+SIZE/2, x+SIZE/2, cvScalarAll(0));
						}
					}
				}

				cvNamedWindow("rec1");
				cvShowImage("rec1", rectifiedImage1);
				cvNamedWindow("rec2");
				cvShowImage("rec2", rectifiedImage2);

				cvReleaseImage(&rectifiedImage1);
				cvReleaseImage(&rectifiedImage2);
			}
			break;
		}
	}
}