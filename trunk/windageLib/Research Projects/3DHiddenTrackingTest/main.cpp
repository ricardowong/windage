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

#define SERVER 1
#define CLIENT 1
#define WRITE_VIDEO 1

const char* IMAGE_FILE_FORMAT = "capture/image_%d_%04d.png";

const int CLIENT_ID = 0;
const int SERVER_ID = 1;
const int NUMBER_OF_IMAGE = 1000;

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const double REAL_WIDTH = 172.0;
const double REAL_HEIGHT = (REAL_WIDTH * 3) / 4;
const double INTRINSIC[] = {1029.275, 1028.858, 322.551, 248.881,-0.206477, 0.306424, 0.000728208, 0.0011338};

const int FEATURE_COUNT = 1000;
const double SCALE_FACTOR = 1.0;
const int SCALE_STEP = 1;
const double REPROJECTION_ERROR = 3.0;

const char* TEMPLATE_IMAGE = "reference%d.png";
const int TEMPLATE_IMAGE_COUNT = 1;

const int OBJECT_COUNT = 1;
const char* FILE_NAME[] = {	"data/reconstruction-2010-11-28_14_05_09/reconstruction.txt", 
							"data/reconstruction-2010-11-28_16_24_23/reconstruction.txt"};

void DrawRectangle(IplImage* image, windage::Calibration* calibration, double dx, double dy, double dz, double x=0, double y=0, double z=0)
{
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
		cvLine(image, pt[i], pt[i2], CV_RGB(0, 0, 0), 5);
		cvLine(image, pt[i+4], pt[i2+4], CV_RGB(0, 0, 0), 5);
		cvLine(image, pt[i], pt[i+4], CV_RGB(0, 0, 0), 5);
	}
	for(int i=0; i<4; i++)
	{
		int i2 = i==3?0:i+1;
		cvLine(image, pt[i], pt[i2], CV_RGB(255, 255, 255), 2);
		cvLine(image, pt[i+4], pt[i2+4], CV_RGB(255, 255, 255), 2);
		cvLine(image, pt[i], pt[i+4], CV_RGB(255, 255, 255), 2);
	}
}

windage::Frameworks::MultiplePlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::MultiplePlanarObjectTracking* tracking = new windage::Frameworks::MultiplePlanarObjectTracking();

	windage::Calibration* calibration;
	windage::Algorithms::FeatureDetector* detector;
	windage::Algorithms::OpticalFlow* opticalflow;
	windage::Algorithms::HomographyEstimator* estimator;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::HomographyRefiner* refiner;

	calibration = new windage::Calibration();
	detector = new windage::Algorithms::SURFdetector();
	opticalflow = new windage::Algorithms::OpticalFlow();
	estimator = new windage::Algorithms::RANSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(50.0);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR * 3);
	refiner->SetMaxIteration(50);

	tracking->AttatchCalibration(calibration);
	tracking->AttatchDetetor(detector);
	tracking->AttatchTracker(opticalflow);
	tracking->AttatchEstimator(estimator);
	tracking->AttatchChecker(checker);
	tracking->AttatchRefiner(refiner);
	
	tracking->Initialize(WIDTH, HEIGHT, (double)REAL_WIDTH, (double)REAL_HEIGHT);
	tracking->SetFilter(false);
	tracking->SetDitectionRatio(2);

	return tracking;
}
windage::Frameworks::MultipleObjectTracking* Create3DTracker()
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
void DrawOutLine(IplImage* colorImage, windage::Calibration* calibration, bool drawCross)
{
	int size = 4;
	CvScalar color = CV_RGB(0, 0, 0);
	CvScalar color2 = CV_RGB(255, 255, 255);
	int width = 640;
	int height = 480;

	cvLine(colorImage, calibration->ConvertWorld2Image(-width/2, -height/2, 0.0),	calibration->ConvertWorld2Image(+width/2, -height/2, 0.0),	color2, size+4);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width/2, -height/2, 0.0),	calibration->ConvertWorld2Image(+width/2, +height/2, 0.0),	color2, size+4);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width/2, +height/2, 0.0),	calibration->ConvertWorld2Image(-width/2, +height/2, 0.0),	color2, size+4);
	cvLine(colorImage, calibration->ConvertWorld2Image(-width/2, +height/2, 0.0),	calibration->ConvertWorld2Image(-width/2, -height/2, 0.0),	color2, size+4);

	cvLine(colorImage, calibration->ConvertWorld2Image(-width/2, -height/2, 0.0),	calibration->ConvertWorld2Image(+width/2, -height/2, 0.0),	color, size);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width/2, -height/2, 0.0),	calibration->ConvertWorld2Image(+width/2, +height/2, 0.0),	color, size);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width/2, +height/2, 0.0),	calibration->ConvertWorld2Image(-width/2, +height/2, 0.0),	color, size);
	cvLine(colorImage, calibration->ConvertWorld2Image(-width/2, +height/2, 0.0),	calibration->ConvertWorld2Image(-width/2, -height/2, 0.0),	color, size);

}

void main()
{
	windage::Logger logger(&std::cout);
	windage::Logger logTrackingInfo("calculation", "txt", true);
	windage::Logger logRestoreInfo("restore", "txt", true);

	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	IplImage* serverImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* clientImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

#if WRITE_VIDEO
	CvVideoWriter* serverWriter = cvCreateVideoWriter("server.avi", CV_FOURCC_DEFAULT, 15, cvSize(WIDTH, HEIGHT));
	CvVideoWriter* clientWriter = cvCreateVideoWriter("client.avi", CV_FOURCC_DEFAULT, 15, cvSize(WIDTH, HEIGHT));
	CvVideoWriter* writer = cvCreateVideoWriter("result.avi", CV_FOURCC_DEFAULT, 15, cvSize(WIDTH, HEIGHT));
	IplImage* resultWriter = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
#endif

	// create and initialize tracker
	double serverThreshold = 30.0;
	double clientThreshold = 30.0;
	windage::Frameworks::MultiplePlanarObjectTracking* serverTracker;
	windage::Frameworks::MultipleObjectTracking* server3DTracker;
	windage::Frameworks::MultiplePlanarObjectTracking* clientTracker;
	serverTracker = CreateTracker();
	server3DTracker = Create3DTracker();
//	serverTracker->AttatchDetetor(new windage::Algorithms::SIFTGPUdetector());
	clientTracker = CreateTracker();
//	clientTracker->AttatchDetetor(new windage::Algorithms::SIFTdetector());

	for(int i=0; i<TEMPLATE_IMAGE_COUNT; i++)
	{
		char message[100];
		sprintf_s(message, TEMPLATE_IMAGE, i+1);
		IplImage* sampleImage = cvLoadImage(message, 0);

		serverTracker->GetDetector()->SetThreshold(30.0);
		serverTracker->AttatchReferenceImage(sampleImage);

		clientTracker->GetDetector()->SetThreshold(30.0);
		clientTracker->AttatchReferenceImage(sampleImage);

		cvReleaseImage(&sampleImage);
	}
	serverTracker->TrainingReference(SCALE_FACTOR, SCALE_STEP);
	serverTracker->GetDetector()->SetThreshold(serverThreshold);
	
	clientTracker->TrainingReference(SCALE_FACTOR, SCALE_STEP);
	clientTracker->GetDetector()->SetThreshold(serverThreshold);
	clientTracker->SetDitectionRatio(2);

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

		server3DTracker->TrainingReference(&referenceRepository);
	}

	std::vector<bool> updated; updated.resize(OBJECT_COUNT+1);
	std::vector<windage::Matrix4> relationList; relationList.resize(OBJECT_COUNT+1);
	std::vector<windage::Matrix3> rotationList; rotationList.resize(OBJECT_COUNT+1);
	std::vector<windage::Vector3> translationList; translationList.resize(OBJECT_COUNT+1);

	windage::Calibration* calibrationTemp = new windage::Calibration();
	calibrationTemp->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	int count;
	int countRest;
	int localcount;
	int serverKeypointCount = 0;
	int clientKeypointCount = 0;
	double processingTime = 0.0;
	std::vector<int> matchingCount;

	char message[100];
	bool processing = true;
	
	for(int k=100; k<NUMBER_OF_IMAGE&&processing; k++)
	{
#if SERVER
		logger.updateTickCount();

		sprintf_s(message, IMAGE_FILE_FORMAT, SERVER_ID, k);
		inputImage = cvLoadImage(message);
		cvResize(inputImage, resizeImage);
		cvReleaseImage(&inputImage);
		
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

		// track server side
		serverTracker->UpdateCamerapose(grayImage);
		server3DTracker->UpdateCamerapose(grayImage);

		// adaptive serverThreshold
		localcount = serverTracker->GetDetector()->GetKeypointsCount();
		if(serverKeypointCount != localcount)
		{
			if(localcount > FEATURE_COUNT)
				serverThreshold += 1;
			if(localcount < FEATURE_COUNT)
				serverThreshold -= 1;
			serverTracker->GetDetector()->SetThreshold(serverThreshold);
			serverKeypointCount = localcount;
		}

		// draw result
		count = 0;
		matchingCount.resize(serverTracker->GetObjectCount());
		for(int i=0; i<serverTracker->GetObjectCount(); i++)
		{
			matchingCount[i] = serverTracker->GetMatchingCount(i);
			if(serverTracker->GetMatchingCount(i) > 10)
			{
//				serverTracker->DrawDebugInfo(resultImage, i);
				serverTracker->DrawOutLine(resultImage, i, true);

				windage::Calibration* calibrationTemp = serverTracker->GetCameraParameter(i);
				calibrationTemp->DrawInfomation(resultImage, 50);
				CvPoint centerPoint = calibrationTemp->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				centerPoint.x += 5;
				centerPoint.y += 10;
				sprintf_s(message, "object #%d (%03d)", i+1, matchingCount[i]);
				windage::Utils::DrawTextToImage(resultImage, centerPoint, 0.6, message);

				count++;
			}
		}
		for(int i=0; i<server3DTracker->GetObjectCount(); i++)
		{
			matchingCount[i] = server3DTracker->GetMatchingCount(i);
			if(server3DTracker->GetMatchingCount(i) > 10)
			{
//				serverTracker->DrawDebugInfo(resultImage, i);
				server3DTracker->DrawDebugInfo(resultImage, i);
				
				windage::Calibration* calibrationTemp = server3DTracker->GetCameraParameter(i);
				calibrationTemp->DrawInfomation(resultImage, 50);
				DrawRectangle(resultImage, calibrationTemp, 180/2, 125/2, 58/2);

				CvPoint centerPoint = calibrationTemp->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				centerPoint.x += 5;
				centerPoint.y += 10;
				sprintf_s(message, "object #%d (%03d)", i+2, matchingCount[i]);
				windage::Utils::DrawTextToImage(resultImage, centerPoint, 0.6, message);

				count++;
			}
		}

		// calcuate relation
//*
		for(int i=0; i<OBJECT_COUNT; i++)
		{
			updated[i] = false;
			if(serverTracker->GetMatchingCount(0) > 10 && serverTracker->GetMatchingCount(i) > 10)
			{
//				windage::Matrix3 rotation = windage::Coordinator::MultiMarkerCoordinator::GetRotation(serverTracker->GetCameraParameter(0), server3DTracker->GetCameraParameter(i-1));
//				windage::Vector3 translation = windage::Coordinator::MultiMarkerCoordinator::GetTranslation(serverTracker->GetCameraParameter(0), server3DTracker->GetCameraParameter(i-1));
//				rotationList[i] = rotation;
//				translationList[i] = translation;

				windage::Matrix4 relation = windage::Coordinator::MultiMarkerCoordinator::GetRelation(serverTracker->GetCameraParameter(0), server3DTracker->GetCameraParameter(i));
				relationList[i] = relation;

				updated[i] = true;
			}
		}

		logger.log(serverTracker->GetCameraParameter(0)->GetExtrinsicMatrix());
		logger.log(server3DTracker->GetCameraParameter(0)->GetExtrinsicMatrix());

//*/
		sprintf_s(message, "Desktop");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, resultImage->height-30), 2.0, message);

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

//		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		sprintf_s(message, "Detection Number : %d", count);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", serverKeypointCount, serverThreshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
//		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

		cvShowImage("Server-Side", resultImage);

		cvCopyImage(resultImage, serverImage);

#if WRITE_VIDEO
		cvWriteToAVI(serverWriter, resultImage);
		cvSetImageROI(resultWriter, cvRect(0, 120, 320, 240));
		cvResize(resultImage, resultWriter);
#endif
		
#endif
#if CLIENT
		logger.updateTickCount();

		sprintf_s(message, IMAGE_FILE_FORMAT, CLIENT_ID, k);
		inputImage = cvLoadImage(message);
		cvResize(inputImage, resizeImage);
		cvReleaseImage(&inputImage);
		
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

		logger.updateTickCount();

		// track server side
		clientTracker->UpdateCamerapose(grayImage);

		// adaptive serverThreshold
		localcount = clientTracker->GetDetector()->GetKeypointsCount();
		if(clientKeypointCount != localcount)
		{
			if(localcount > FEATURE_COUNT)
				clientThreshold += 1;
			if(localcount < FEATURE_COUNT)
				clientThreshold -= 1;
			clientTracker->GetDetector()->SetThreshold(clientThreshold);
			clientKeypointCount = localcount;
		}

		// draw result
		count = 0;
		matchingCount.resize(clientTracker->GetObjectCount());
		for(int i=0; i<1/*clientTracker->GetObjectCount()*/; i++)
		{
			matchingCount[i] = clientTracker->GetMatchingCount(i);
			if(clientTracker->GetMatchingCount(i) > 10)
			{
				windage::Calibration* calibrationTemp = clientTracker->GetCameraParameter(i);
				if(i == 0)
				{
//					clientTracker->DrawDebugInfo(resultImage, i);
					clientTracker->DrawOutLine(resultImage, i, true);
					calibrationTemp->DrawInfomation(resultImage, 50);
				}
			
				CvPoint centerPoint = calibrationTemp->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				centerPoint.x += 5;
				centerPoint.y += 10;
				sprintf_s(message, "object #%d (%03d)", i+1, matchingCount[i]);
				windage::Utils::DrawTextToImage(resultImage, centerPoint, 0.6, message);

				CvScalar cameraPosition = calibrationTemp->GetCameraPosition();
				logTrackingInfo.log("x", cameraPosition.val[0]);
				logTrackingInfo.log("y", cameraPosition.val[1]);
				logTrackingInfo.log("z", cameraPosition.val[2]);

				count++;
			}
		}
		logTrackingInfo.logNewLine();

		// calcuate relation
		countRest = 1;
		for(int i=0; i<OBJECT_COUNT; i++)
		{
			if(updated[i])
			{
				windage::Matrix4 extrinsic;
//				windage::Matrix4 extrinsic = windage::Coordinator::MultiMarkerCoordinator::CalculateExtrinsic(clientTracker->GetCameraParameter(0), rotationList[i], translationList[i]);
//				clientTracker->GetCameraParameter(i)->SetExtrinsicMatrix(extrinsic.m1);

				extrinsic = windage::Coordinator::MultiMarkerCoordinator::CalculateExtrinsic(clientTracker->GetCameraParameter(0), relationList[i]);
//				extrinsic = extrinsic.Inverse();
//				clientTracker->GetCameraParameter(i)->SetExtrinsicMatrix(extrinsic.m1);
				calibrationTemp->SetExtrinsicMatrix(extrinsic.m1);
				
//				DrawOutLine(resultImage, calibrationTemp, true);
				calibrationTemp->DrawInfomation(resultImage, 50);
				DrawRectangle(resultImage, calibrationTemp, 180/2, 125/2, 58/2, 10, -15, -58/2);

				CvPoint centerPoint = calibrationTemp->ConvertWorld2Image(0.0, 0.0, 0.0);
				
				centerPoint.x += 5;
				centerPoint.y += 10;
				sprintf_s(message, "restoration object #%d", i+1);
				windage::Utils::DrawTextToImage(resultImage, centerPoint, 0.6, message);
				
				CvScalar cameraPosition = calibrationTemp->GetCameraPosition();
				logRestoreInfo.log("x", cameraPosition.val[0]);
				logRestoreInfo.log("y", cameraPosition.val[1]);
				logRestoreInfo.log("z", cameraPosition.val[2]);

				countRest++;
			}
			else
			{
				logRestoreInfo.log("x", -1);
				logRestoreInfo.log("y", -1);
				logRestoreInfo.log("z", -1);
			}
		}
		logRestoreInfo.logNewLine();

		sprintf_s(message, "Mobile device");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, resultImage->height-30), 2.0, message);

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

//		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		sprintf_s(message, "Detection Number : %d, Restoration Number : %d", count, countRest);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", serverKeypointCount, serverThreshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
//		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

		cvShowImage("Client-Side", resultImage);

		cvCopyImage(resultImage, clientImage);

#if WRITE_VIDEO
		cvWriteToAVI(clientWriter, resultImage);

		cvSetImageROI(resultWriter, cvRect(320, 120, 320, 240));
		cvResize(resultImage, resultWriter);
		cvResetImageROI(resultWriter);
		cvWriteToAVI(writer, resultWriter);
#endif
#endif
		logger.log("Frame", k);
		logger.logNewLine();

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			cvWaitKey();
			break;
		case 's':
		case 'S':
			cvSaveImage("server.png", serverImage);
			cvSaveImage("client.png", clientImage);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
 		}		
	}

#if WRITE_VIDEO
	cvReleaseVideoWriter(&serverWriter);
	cvReleaseVideoWriter(&clientWriter);
	cvReleaseVideoWriter(&writer);
#endif

	cvDestroyAllWindows();
}
