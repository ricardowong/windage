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

#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <windage.h>
#include "../Common/FleaCamera.h"

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;

const double REPROJECTION_ERROR = 5.0;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

const int OBJECT_COUNT = 2;
const char* FILE_NAME[] = {	"data/reconstruction-2010-11-28_14_05_09/reconstruction.txt", 
							"data/reconstruction-2010-11-28_16_24_23/reconstruction.txt"};
const char* BASE_IMAGE_NAME = "reference1_320.png";
const char* BASE_FEATURE_NAME = {"data/descriptor-2010-04-26.txt"};

void DrawRectangle(IplImage* image, windage::Calibration* calibration, CvScalar color, double dx, double dy, double dz, double x=0, double y=0, double z=0)
{
	CvScalar tempCP = calibration->GetCameraPosition();
	windage::Vector3 center = windage::Vector3(tempCP.val[0], tempCP.val[1], tempCP.val[2]);
	int farIndex = -1;

	windage::Vector3 tempPt[8];
	tempPt[0] = windage::Vector3(+dx + x, +dy + y, +dz + z);
	tempPt[1] = windage::Vector3(+dx + x, -dy + y, +dz + z);
	tempPt[2] = windage::Vector3(-dx + x, -dy + y, +dz + z);
	tempPt[3] = windage::Vector3(-dx + x, +dy + y, +dz + z);
					   					 
	tempPt[4] = windage::Vector3(+dx + x, +dy + y, -dz + z);
	tempPt[5] = windage::Vector3(+dx + x, -dy + y, -dz + z);
	tempPt[6] = windage::Vector3(-dx + x, -dy + y, -dz + z);
	tempPt[7] = windage::Vector3(-dx + x, +dy + y, -dz + z);

	double farDistance = 0;
	for(int i=0; i<8; i++)
	{
		double distance = center.getDistance(tempPt[i]);
		if(farDistance < distance)
		{
			farDistance = distance;
			farIndex = i;
		}
	}

	CvPoint pt[8];
	pt[0] = calibration->ConvertWorld2Image(+dx + x, +dy + y, +dz + z);
	pt[1] = calibration->ConvertWorld2Image(+dx + x, -dy + y, +dz + z);
	pt[2] = calibration->ConvertWorld2Image(-dx + x, -dy + y, +dz + z);
	pt[3] = calibration->ConvertWorld2Image(-dx + x, +dy + y, +dz + z);
																 
	pt[4] = calibration->ConvertWorld2Image(+dx + x, +dy + y, -dz + z);
	pt[5] = calibration->ConvertWorld2Image(+dx + x, -dy + y, -dz + z);
	pt[6] = calibration->ConvertWorld2Image(-dx + x, -dy + y, -dz + z);
	pt[7] = calibration->ConvertWorld2Image(-dx + x, +dy + y, -dz + z);
	
	for(int i=0; i<4; i++)
	{
		int i2 = i==3?0:i+1;

		if(i != farIndex && i2 != farIndex)
		{
			cvLine(image, pt[i], pt[i2], CV_RGB(255, 255, 255), 5);
		}
		if(i+4 != farIndex && i2+4 != farIndex)
		{
			cvLine(image, pt[i+4], pt[i2+4], CV_RGB(255, 255, 255), 5);
		}
		if(i != farIndex && i+4 != farIndex)
		{
			cvLine(image, pt[i], pt[i+4], CV_RGB(255, 255, 255), 5);
		}
	}
	for(int i=0; i<4; i++)
	{
		int i2 = i==3?0:i+1;

		if(i != farIndex && i2 != farIndex)
			cvLine(image, pt[i], pt[i2], color, 2);
		if(i+4 != farIndex && i2+4 != farIndex)
			cvLine(image, pt[i+4], pt[i2+4], color, 2);
		if(i != farIndex && i+4 != farIndex)
			cvLine(image, pt[i], pt[i+4], color, 2);
	}
}

windage::Frameworks::MultipleObjectTracking* CreateMultipleObjectTracking()
{
	windage::Frameworks::MultipleObjectTracking* tracking = new windage::Frameworks::MultipleObjectTracking();
	
	windage::Calibration* calibration					= new windage::Calibration();
	windage::Algorithms::OpticalFlow* opticalflow		= new windage::Algorithms::OpticalFlow();
	windage::Algorithms::OpenCVRANSACestimator* estimator	= new windage::Algorithms::OpenCVRANSACestimator();
	windage::Algorithms::PoseRefiner* refiner			= new windage::Algorithms::PoseLMmethod();
	
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	estimator->SetConfidence(0.95);
	estimator->SetMaxIteration(50);
	refiner->SetMaxIteration(5);

	tracking->AttatchCalibration(calibration);
	tracking->AttatchTracker(opticalflow);
	tracking->AttatchEstimator(estimator);
	tracking->AttatchRefiner(refiner);

	tracking->SetDitectionRatio(3);
	tracking->Initialize(WIDTH, HEIGHT);
	return tracking;
}

windage::Frameworks::PlanarObjectTracking* CreatePlanerTracking()
{
	windage::Frameworks::PlanarObjectTracking* tracking = new windage::Frameworks::PlanarObjectTracking();
	
	windage::Calibration* calibration					= new windage::Calibration();
	windage::Algorithms::WSURFdetector* detector		= new windage::Algorithms::WSURFdetector();
	windage::Algorithms::KDtree* tree					= new windage::Algorithms::KDtree();
	windage::Algorithms::OpticalFlow* opticalflow		= new windage::Algorithms::OpticalFlow();
	windage::Algorithms::RANSACestimator* estimator		= new windage::Algorithms::RANSACestimator();
	windage::Algorithms::OutlierChecker* checker		= new windage::Algorithms::OutlierChecker();
	windage::Algorithms::HomographyRefiner* refiner		= new windage::Algorithms::LMmethod();
	
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(30.0);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	refiner->SetMaxIteration(5);

	tracking->AttatchDetetor(detector);
	tracking->AttatchMatcher(tree);
	tracking->AttatchCalibration(calibration);
	tracking->AttatchTracker(opticalflow);
	tracking->AttatchEstimator(estimator);
	tracking->AttatchChecker(checker);
	tracking->AttatchRefiner(refiner);

	tracking->SetDitectionRatio(3);
	tracking->Initialize(WIDTH, HEIGHT);
	return tracking;
}



void main()
{
	windage::Logger logger(&std::cout);

	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	CvVideoWriter* writer = NULL;
	FleaCamera* capture = new FleaCamera();
	capture->open();
	capture->start();
//	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	// create and initialize tracker
	windage::Frameworks::MultipleObjectTracking* tracking;
	windage::Frameworks::PlanarObjectTracking* baseTracking;

	tracking = CreateMultipleObjectTracking();
	baseTracking = CreatePlanerTracking();

	int keypointCount = 0;
	int matchingCount = 0;
	double processingTime = 0.0;
	bool trained = false;

	// load tracking data
	std::vector<windage::FeaturePoint> referenceRepository;

	std::vector<windage::Calibration*> calibrationList;
	std::vector<std::string> filenameList;
	std::vector<windage::ReconstructionPoint> reconstructionPoints;

	windage::Reconstruction::Loader* loader = new windage::Reconstruction::Loader();
	loader->AttatchCalibration(&calibrationList);
	loader->AttatchFilename(&filenameList);
	loader->AttatchReconstructionPoints(&reconstructionPoints);

	for(int i=0; i<OBJECT_COUNT; i++)
	{
		referenceRepository.clear();
		reconstructionPoints.clear();
		loader->DoLoad(FILE_NAME[i]);

		for(unsigned int i=0; i<reconstructionPoints.size(); i++)
		{
			int index = cvRound((double)reconstructionPoints[i].GetFeatureList()->size() / 2.0);
			windage::FeaturePoint feature = reconstructionPoints[i].GetFeature(index);
			windage::Vector4 point = reconstructionPoints[i].GetPoint();
			feature.SetPoint(windage::Vector3(point.x, point.y, point.z));
			feature.SetObjectID(0);
			feature.SetRepositoryID(i);

			referenceRepository.push_back(feature);
		}

		tracking->TrainingReference(&referenceRepository);
	}

	IplImage* baseImage = cvLoadImage(BASE_IMAGE_NAME, 0);
	baseTracking->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);
	baseTracking->AttatchReferenceImage(baseImage);
	baseTracking->TrainingReference(4.0, 8);
/*
	std::vector<windage::FeaturePoint> baseFeaturePoints;
	
	windage::FeatureLoader* baseLoader = new windage::FeatureLoader();
	baseLoader->AttatchFeaturePoints(&baseFeaturePoints);
	baseLoader->DoLoad(BASE_FEATURE_NAME);

	for(unsigned int i=0; i<baseFeaturePoints.size(); i++)
	{
		windage::Vector3 pt = baseFeaturePoints[i].GetPoint();
		pt /= pt.z;
		pt.z = 0.0;
		baseFeaturePoints[i].SetPoint(pt);
	}
//	tracking->TrainingReference(&baseFeaturePoints);
//*/	
	trained = true;

	char message[100];
	bool saving = false;
	bool flip = true;
	bool processing = true;
	while(processing)
	{
		// capture image
		// capture image
		capture->update();
		inputImage = capture->GetIPLImage();
		cvCvtColor(inputImage, resizeImage, CV_BGRA2BGR);
/*
		inputImage = cvRetrieveFrame(capture);
		if(flip)
			cvFlip(inputImage, inputImage);
		cvResize(inputImage, resizeImage);
*/
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

		logger.updateTickCount();

		// track object
		if(trained)
		{
			#pragma omp sections
			{
				#pragma omp section
				{
					tracking->UpdateCamerapose(grayImage);
				}
				#pragma omp section
				{
					baseTracking->UpdateCamerapose(grayImage);
				}
			}
			matchingCount = 0;

			// draw result
			for(int i=0; i<tracking->GetObjectCount(); i++)
			{
				matchingCount += tracking->GetMatchingCount(i);
				int matchedCount = tracking->GetMatchingCount(i);
				if(matchedCount > 9)
				{
//					tracking->DrawDebugInfo(resultImage, i);
					
					windage::Calibration* cameraParam = tracking->GetCameraParameter(i);
					cameraParam->DrawInfomation(resultImage, 50);
					if(i == 0)
						DrawRectangle(resultImage, cameraParam, CV_RGB(255, 0, 255), 180/2, 125/2, 58/2, 5, 0, -5);
					else if(i==1)
						DrawRectangle(resultImage, cameraParam, CV_RGB(255, 0, 255), 180/2, 80/2, 58/2, 0, -5, -10);

					CvPoint point = cameraParam->ConvertWorld2Image(0.0, 0.0, 0.0);
					point.x += 5;
					point.y += 10;
					sprintf_s(message, "object #%d (%03d)", i+1, matchedCount);

					windage::Utils::DrawTextToImage(resultImage, point, 0.6, message);
				}
			}

//			baseTracking->DrawDebugInfo(resultImage);
			baseTracking->DrawOutLine(resultImage, true);
			baseTracking->GetCameraParameter()->DrawInfomation(resultImage, 100);
		}
//		matchingCount = tracking->GetMatchingCount();

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);

		if(saving == true)
		{
			cvWriteFrame(writer, resultImage);
		}

		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		case 's':
		case 'S':
			writer = cvCreateVideoWriter("result.avi", CV_FOURCC_DEFAULT, 30.0, cvSize(WIDTH, HEIGHT));
			saving = !saving;
			break;
		case 'f':
		case 'F':
			flip = !flip;
			break;
		}		
	}

	if(writer) cvReleaseVideoWriter(&writer);
//	cvReleaseCapture(&capture);
	capture->stop();
	capture->close();
	cvDestroyAllWindows();
}
