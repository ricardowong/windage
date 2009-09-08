#define RUNNING
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

#define FLIP
#define RECTIFICATION

const int WIDTH = 640;
const int HEIGHT = 480;

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	cvNamedWindow("result");
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);

	// Tracker Initialize
	IplImage* referenceImage = cvLoadImage("reference.png", 0);

	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 30);
	tracker->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 6);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	windage::Calibration* savedCalibration = new windage::Calibration();
	savedCalibration->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114);

	// for opticalFlow
	windage::OpticalFlow* opticalFlow = new windage::OpticalFlow();
	opticalFlow->Initialize(WIDTH, HEIGHT);
	
	IplImage* prevImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	std::vector<CvPoint2D32f> savedPoints;
	std::vector<CvPoint2D32f> prevPoints;
	std::vector<CvPoint2D32f> currPoints;

	// for Reconstruction
	std::vector<CvScalar> worldPoints;

	// for extended tracker
	std::vector<windage::SURFFeature*> featureList;
	std::vector<windage::SURFFeature*> referenceList;
	windage::FeatureTree* featureTree = new windage::FeatureTree();	

	char message[100];

	fpslog->updateTickCount();

	bool isExtendedTracking = false;
	bool isReconstruction = false;
	bool isOpticalFlow = false;
	bool processing = true;
	while(processing)
	{
		double fps = fpslog->calculateFPS();
		fpslog->updateTickCount();
		
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		IplImage* grabFrame = cvQueryFrame(capture);
#ifdef FLIP
		cvFlip(grabFrame, grabFrame);
#endif
#ifdef RECTIFICATION
		tracker->GetCameraParameter()->Undistortion(grabFrame, inputImage);
#else
		cvCopy(grabFrame, inputImage);
#endif

		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		int result = tracker->UpdateCameraPose(grayImage);

		// draw tracking result
//		tracker->DrawDebugInfo(inputImage);
		tracker->DrawOutLine(inputImage);
		tracker->DrawInfomation(inputImage, 10.0);

		log->log("tracking", log->calculateProcessTime());
		log->log("featureCount", result);
		log->logNewLine();

		// optical flow
		if(isOpticalFlow)
		{
			currPoints.clear();
			opticalFlow->TrackFeature(prevImage, grayImage, &prevPoints, &currPoints);

			prevPoints.clear();
			for(int i=0; i<currPoints.size(); i++)
			{
				if(currPoints[i].x > 0 && currPoints[i].y > 0)
				{
					cvLine(inputImage, cvPoint(savedPoints[i].x, savedPoints[i].y), cvPoint(currPoints[i].x, currPoints[i].y), CV_RGB(255, 0, 255));
				}

				prevPoints.push_back(currPoints[i]);
			}
		}
		cvCopyImage(grayImage, prevImage);

		// reconstruction
		if(isReconstruction)
		{
			for(int i=0; i<worldPoints.size(); i++)
			{
				windage::Utils::DrawWorldCoordinatePoint(inputImage, tracker->GetCameraParameter(), worldPoints[i]);

//				CvPoint projectionPoint = tracker->GetCameraParameter()->ConvertWorld2Image(worldPoints[i].val[0], worldPoints[i].val[1], worldPoints[i].val[2]);
//				cvRectangle(inputImage, cvPoint(projectionPoint.x - 3, projectionPoint.y - 3), cvPoint(projectionPoint.x + 3, projectionPoint.y + 3), CV_RGB(0, 255, 0));
			}
		}

		// extanted tracking
		if(isExtendedTracking)
		{
			std::vector<CvPoint> fastPoints;
			windage::FeatureFactory::ExtractFASTCorner(grayImage, 60, &fastPoints);
			windage::FeatureFactory::Create2DPlaneSURFFeatureDescriptor(grayImage, &fastPoints, &featureList, 1.0, 1);

			for(int i=0; i<featureList.size(); i++)
			{
				int index = featureTree->FindPairs(featureList[i]);
				if(index > 0)
				{
					index /= referenceList[0]->GetDescriptionList()->size();

					CvPoint point1 = cvPoint(featureList[i]->GetPosition().x, featureList[i]->GetPosition().y);
					CvPoint point2 = tracker->GetCameraParameter()->ConvertWorld2Image(referenceList[index]->GetPosition().x, referenceList[index]->GetPosition().y, referenceList[index]->GetPosition().z);

					if(abs(point1.x - point2.x) + abs(point1.x - point2.x) < 5)
						cvCircle(inputImage, point1, 5, CV_RGB(255, 0, 0), 5);
//					cvLine(inputImage, point1, point2, CV_RGB(255, 0, 255));
				}
			}

			for(int i=0; i<featureList.size(); i++)
			{
				delete featureList[i];
				featureList[i] = NULL;
			}
			featureList.clear();
		}

		sprintf(message, "FPS : %lf, Feature Count : %d", fps, result);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), message);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'a':
		case 'A':
			{
				std::cout << "extract feature and tracking" << std::endl;
				if(isOpticalFlow)
				{
					isOpticalFlow = false;
				}
				else
				{
					savedCalibration->SetExtrinsicMatrix(tracker->GetCameraParameter()->GetExtrinsicMatrix());

					std::vector<CvPoint> points;
					windage::ModifiedSURFTracker::ExtractFASTCorner(&points, grayImage, 45);

					CvPoint corner1 = tracker->GetCameraParameter()->ConvertWorld2Image(-tracker->GetRealWidth()/2, -tracker->GetRealHeight()/2, 0.0);
					CvPoint corner2 = tracker->GetCameraParameter()->ConvertWorld2Image(+tracker->GetRealWidth()/2, -tracker->GetRealHeight()/2, 0.0);
					CvPoint corner3 = tracker->GetCameraParameter()->ConvertWorld2Image(+tracker->GetRealWidth()/2, +tracker->GetRealHeight()/2, 0.0);
					CvPoint corner4 = tracker->GetCameraParameter()->ConvertWorld2Image(-tracker->GetRealWidth()/2, +tracker->GetRealHeight()/2, 0.0);

					savedPoints.clear();
					prevPoints.clear();
					for(int i=0; i<points.size(); i++)
					{
						if(windage::Utils::IsInside(points[i], corner1, corner2, corner3, corner4, true) == false)
						{
							savedPoints.push_back(cvPoint2D32f(points[i].x, points[i].y));
							prevPoints.push_back(cvPoint2D32f(points[i].x, points[i].y));
						}
					}

					isOpticalFlow = true;
				}
			}
			break;
		case 'p':
		case 'P':
			{
				std::cout << "reconstruct 3d points" << std::endl;
				if(isReconstruction)
				{
					isReconstruction = false;
				}
				else
					{
					worldPoints.clear();
					for(int i=0; i<currPoints.size(); i++)
					{
						if(currPoints[i].x > 0 && currPoints[i].y > 0)
						{
							CvPoint point1 = cvPoint(savedPoints[i].x, savedPoints[i].y);
							CvPoint point2 = cvPoint(prevPoints[i].x, prevPoints[i].y);

							CvScalar worldPoint = windage::Reconstructor::Calc3DPointApproximation(savedCalibration, tracker->GetCameraParameter(), point1, point2);
							worldPoints.push_back(worldPoint);
						}
					}

					isReconstruction = true;
					isOpticalFlow = false;
				}
			}
			break;
		case 'r':
		case 'R':
			{
				std::cout << "generate reference descriptor" << std::endl;
				for(int i=0; i<worldPoints.size(); i++)
				{
					windage::SURFFeature* tempFeature = new windage::SURFFeature();
					tempFeature->initialize(3.0, 6);
					tempFeature->SetPosition(windage::Vector3(worldPoints[i].val[0], worldPoints[i].val[1], worldPoints[i].val[2]));

					CvPoint projectionPoint = tracker->GetCameraParameter()->ConvertWorld2Image(worldPoints[i].val[0], worldPoints[i].val[1], worldPoints[i].val[2]);
					int result = tempFeature->GenerateDescriptor(grayImage, projectionPoint);

					if(result > 0)
						referenceList.push_back(tempFeature);
					else
						delete tempFeature;
				}
				featureTree->CreateReferenceTree(&referenceList);

				isExtendedTracking = true;
				isReconstruction = false;
			}
			break;
		case 'e':
		case 'E':
			{
				std::cout << "plane estimation" << std::endl;

				windage::Vector3 center;
				std::vector<windage::Vector3> dataPoints;
				std::vector<windage::Vector3> consensusPoints;
				for(int i=0; i<worldPoints.size(); i++)
					dataPoints.push_back(windage::Vector3(worldPoints[i].val[0], worldPoints[i].val[1], worldPoints[i].val[2]));

				windage::Reconstructor::PlaneEstimationRANSAC(&dataPoints, center, &consensusPoints);
				worldPoints.clear();
				for(int i=0; i<consensusPoints.size(); i++)
					worldPoints.push_back(cvScalar(consensusPoints[i].x, consensusPoints[i].y, consensusPoints[i].z));
			}
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
	}

	cvReleaseCapture(&capture);
}

#endif