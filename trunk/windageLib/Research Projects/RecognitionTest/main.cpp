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

#include <gl/glut.h>
#include <cv.h>
#include <highgui.h>

#include <windage.h>

#define SIFT 0
#define SURF 0
#define WSURF 1

// 1, 5, 6
#define REFERENCE_NUMBER 1
#define PERFORMANCE 0
#define ACCURACY 0
#define MEMORY 1

char FILE_NAME[1000];
const char* FILE_NAME_TEMPLATE = "testImages\\reference%d.png";
const char* RESULT_DIR = "testImages\\reference%d_d-%d_%c-%d.png";
const char* RESULT_SAVE_DIR = "testImagesResult\\reference%d_d-%d_%c-%d_result.png";
const int WIDTH = 640;
const int HEIGHT = 480;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};
//const double INTRINSIC[] = {1000.0, 1000.0, 320, 240, 0, 0, 0, 0};

const int ROTATION_RANGE_S = 0;
const int ROTATION_RANGE_E = +80;
const int ROTATION_RANGE_D = 10;

const int DISTANCE_RANGE_S = -600;
const int DISTANCE_RANGE_E = 2000;
const int DISTANCE_RANGE_D = 300;

#if WSURF
const double SCALE_FACTOR = 4.0;
const int SCALE_STEP = 10;
#else
const double SCALE_FACTOR = 1.0;
const int SCALE_STEP = 1;
#endif
const double REPROJECTION_ERROR = 5.0;

windage::Vector3 EstimateCameraPose(int rMode, double d, double theta)
{
	windage::Vector3 result;
	double radius = theta * CV_PI / 180;
	switch(rMode)
	{
	case 0://x
		result.x = 0;
		result.y = -sin(radius) * d;
		result.z = cos(radius) * d;
		break;
	case 1://y
		result.x = sin(radius) * d;
		result.y = 0;
		result.z = cos(radius) * d;
		break;
	case 2://z
		result.x = 0;
		result.y = 0;
		result.z = d;
		break;
	}

	return result;
}

void main()
{
	windage::Logger logger(&std::cout);
#if ACCURACY
	char loggerFileX[100], loggerFileY[100], loggerFileZ[100];
#if SIFT
	sprintf(loggerFileX, "SIFT_recognition%d_x", REFERENCE_NUMBER);
	sprintf(loggerFileY, "SIFT_recognition%d_y", REFERENCE_NUMBER);
	sprintf(loggerFileZ, "SIFT_recognition%d_z", REFERENCE_NUMBER);
#elif SURF
	sprintf(loggerFileX, "SURF_recognition%d_x", REFERENCE_NUMBER);
	sprintf(loggerFileY, "SURF_recognition%d_y", REFERENCE_NUMBER);
	sprintf(loggerFileZ, "SURF_recognition%d_z", REFERENCE_NUMBER);
#else
	sprintf(loggerFileX, "WSURF_recognition%d_x", REFERENCE_NUMBER);
	sprintf(loggerFileY, "WSURF_recognition%d_y", REFERENCE_NUMBER);
	sprintf(loggerFileZ, "WSURF_recognition%d_z", REFERENCE_NUMBER);
#endif
	
	windage::Logger fileXLog(//&std::cout);
							 loggerFileX, "txt", false);
	windage::Logger fileYLog(//&std::cout);
							 loggerFileY, "txt", false);
	windage::Logger fileZLog(//&std::cout);
							 loggerFileZ, "txt", false);
#elif PERFORMANCE
	char perfilename[100];
#if SIFT
	sprintf(perfilename, "SIFT_recognition%d_p", REFERENCE_NUMBER);
#elif SURF
	sprintf(perfilename, "SURF_recognition%d_p", REFERENCE_NUMBER);
#else
	sprintf(perfilename, "WSURF_recognition%d_p", REFERENCE_NUMBER);
#endif
	windage::Logger performance(perfilename, "txt", false);
#endif

	sprintf(FILE_NAME, FILE_NAME_TEMPLATE, REFERENCE_NUMBER);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	cvNamedWindow("result");

	// create and initialize tracker
	windage::Frameworks::PlanarObjectTracking tracking;
	windage::Calibration* calibration;
	windage::Algorithms::FeatureDetector* detector;
	windage::Algorithms::SearchTree* searchtree;
	windage::Algorithms::HomographyEstimator* estimator;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::HomographyRefiner* refiner;

	calibration = new windage::Calibration();
#if SIFT
	detector = new windage::Algorithms::SIFTdetector();
#elif SURF
	detector = new windage::Algorithms::SURFdetector();
#else
	detector = new windage::Algorithms::WSURFdetector();
#endif
	
	searchtree = new windage::Algorithms::FLANNtree();
	estimator = new windage::Algorithms::RANSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
//	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR * 3);
	refiner->SetMaxIteration(10);

	tracking.AttatchCalibration(calibration);
	tracking.AttatchDetetor(detector);
	tracking.AttatchMatcher(searchtree);
	tracking.AttatchEstimator(estimator);
	tracking.AttatchChecker(checker);
	tracking.AttatchRefiner(refiner);
	tracking.SetDitectionRatio(-1);

#if PERFORMANCE
	tracking.SetPerformance(&performance);
#endif

	tracking.Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	IplImage* sampleImage = cvLoadImage(FILE_NAME, 0);
//	cvSmooth(sampleImage, sampleImage);

	logger.updateTickCount();
	tracking.AttatchReferenceImage(sampleImage);
	tracking.TrainingReference(SCALE_FACTOR, SCALE_STEP);
	logger.log("training time", logger.calculateProcessTime());
	logger.logNewLine();
	logger.log("training size", (int)tracking.GetReferenceRep()->size());
	logger.logNewLine();

	windage::Vector3 cameraPositionE;
	windage::Vector3 cameraPositionC;

	char message[1000];
	char filename[1000];
	char filenameSave[1000];
	bool processing = true;
	double processingTime;
	while(processing)
	{
//		cvWaitKey(0);

		for(int d=DISTANCE_RANGE_S; d<=DISTANCE_RANGE_E; d+=DISTANCE_RANGE_D)
		{
			for(int rMode=0; rMode<3; rMode++)
			{
				for(int r=ROTATION_RANGE_S; r<=ROTATION_RANGE_E; r+=ROTATION_RANGE_D)
				{
/*
//					int rMode, r;
					d = DISTANCE_RANGE_S + 900;
					rMode = 0;
					r = 0;
//*/
					int tempR = (r<0)?360+r:r;
					int tempD = d - DISTANCE_RANGE_S;


					switch(rMode)
					{
					case 0://x
						sprintf(filename, RESULT_DIR, REFERENCE_NUMBER, tempD, 'x', tempR);
						sprintf(filenameSave, RESULT_SAVE_DIR, REFERENCE_NUMBER, tempD, 'x', tempR);
						break;
					case 1://y
						sprintf(filename, RESULT_DIR, REFERENCE_NUMBER, tempD, 'y', tempR);
						sprintf(filenameSave, RESULT_SAVE_DIR, REFERENCE_NUMBER, tempD, 'y', tempR);
						break;
					case 2://z
						sprintf(filename, RESULT_DIR, REFERENCE_NUMBER, tempD, 'z', tempR);
						sprintf(filenameSave, RESULT_SAVE_DIR, REFERENCE_NUMBER, tempD, 'z', tempR);
						break;
					}
#if ACCURACY
					switch(rMode)
					{
					case 0://x
						tracking.SetLogger(&fileXLog);
						fileXLog.log("D", d);
						fileXLog.log("R", r);
						break;
					case 1://y
						fileYLog.log("D", d);
						fileYLog.log("R", r);
						tracking.SetLogger(&fileYLog);
						break;
					case 2://z
						tracking.SetLogger(&fileZLog);
						fileZLog.log("D", d);
						fileZLog.log("R", r);
						break;
					}

					cameraPositionE = EstimateCameraPose(rMode, (double)d+INTRINSIC[0], (double)r);
#endif
					// capture image
					IplImage* inputImage = cvLoadImage(filename);
//					cvSmooth(inputImage, inputImage);

					cvCopyImage(inputImage, resultImage);
					cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);

					cvReleaseImage(&inputImage);

					logger.updateTickCount();

					// track object
					tracking.UpdateCamerapose(grayImage);

					// draw result
//					detector->DrawKeypoints(resultImage);

					if(tracking.GetMatchingCount() > 10)
					{
						tracking.DrawOutLine(resultImage, true);
//					tracking.DrawDebugInfo(resultImage);
						calibration->DrawInfomation(resultImage, 100);
						
					}

#if ACCURACY
					cameraPositionC = windage::Vector3();
					CvScalar tempPos = calibration->GetCameraPosition();

					cameraPositionC.x = tempPos.val[0];
					cameraPositionC.y = tempPos.val[1];
					cameraPositionC.z = tempPos.val[2];

					cameraPositionC -= cameraPositionE;
					cameraPositionC.x = fabs(cameraPositionC.x);
					cameraPositionC.y = fabs(cameraPositionC.y);
					cameraPositionC.z = fabs(cameraPositionC.z);

					switch(rMode)
					{
					case 0://x
						fileXLog.log("poseX", cameraPositionC.x);
						fileXLog.log("poseY", cameraPositionC.y);
						fileXLog.log("poseZ", cameraPositionC.z);
						fileXLog.logNewLine();
						break;
					case 1://y
						fileYLog.log("poseX", cameraPositionC.x);
						fileYLog.log("poseY", cameraPositionC.y);
						fileYLog.log("poseZ", cameraPositionC.z);
						fileYLog.logNewLine();
						break;
					case 2://z
						fileZLog.log("poseX", cameraPositionC.x);
						fileZLog.log("poseY", cameraPositionC.y);
						fileZLog.log("poseZ", cameraPositionC.z);
						fileZLog.logNewLine();
						break;
					}
#endif
					int keypointCount = detector->GetKeypointsCount();
					int matchingCount = tracking.GetMatchingCount();

					processingTime = logger.calculateProcessTime();
					logger.log("processingTime", processingTime);
					logger.logNewLine();

					sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
//					windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
					sprintf_s(message, "Feature Count : %d", keypointCount);
//					windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
					sprintf_s(message, "Matching Count : %d", matchingCount);
//					windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

					sprintf_s(message, "File name : %s", filename);
					windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-350, HEIGHT-10), 0.5, message);
					cvShowImage("result", resultImage);

					char ch = cvWaitKey(1);
					switch(ch)
					{
					case 's':
					case 'S':
						std::cout << "file sasve : " << filenameSave << std::endl;
						cvSaveImage(filenameSave, resultImage);
						break;
					case 'q':
					case 'Q':
						processing = false;
						break;
					}
				}
			}
		}
		processing = false;
	}

	cvDestroyAllWindows();
}

