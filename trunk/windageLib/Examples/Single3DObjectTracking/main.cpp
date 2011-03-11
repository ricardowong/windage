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

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;

const double REPROJECTION_ERROR = 2.0;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

//const char* FILE_NAME = "data/reconstruction-2010-03-29_18_28_38/reconstruction.txt";
//const char* FILE_NAME = "data/reconstruction-2010-03-29_09_33_01/reconstruction.txt";
//const char* FILE_NAME = "data/reconstruction-2010-03-29_09_33_01/reconstruction.txt";
const char* FILE_NAME = "data/reconstruction-2010-11-28_23_08_28/reconstruction.txt";

void DrawRectangle(IplImage* image, windage::Calibration* calibration, double dx, double dy, double dz)
{
	double y = dy;
	CvPoint pt[8];

	pt[0] = calibration->ConvertWorld2Image(+dx, +dy + y, +dz);
	pt[1] = calibration->ConvertWorld2Image(+dx, -dy + y, +dz);
	pt[2] = calibration->ConvertWorld2Image(-dx, -dy + y, +dz);
	pt[3] = calibration->ConvertWorld2Image(-dx, +dy + y, +dz);

	pt[4] = calibration->ConvertWorld2Image(+dx, +dy + y, -dz);
	pt[5] = calibration->ConvertWorld2Image(+dx, -dy + y, -dz);
	pt[6] = calibration->ConvertWorld2Image(-dx, -dy + y, -dz);
	pt[7] = calibration->ConvertWorld2Image(-dx, +dy + y, -dz);
	
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

void main()
{
	windage::Logger logger(&std::cout);

	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	CvVideoWriter* writer = NULL;
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	// create and initialize tracker
	windage::Frameworks::SingleObjectTracking tracking;
	windage::Calibration* calibration					= new windage::Calibration();
	windage::Algorithms::SearchTree* searchtree			= new windage::Algorithms::FLANNtree(30);
	windage::Algorithms::OpticalFlow* opticalflow		= new windage::Algorithms::OpticalFlow();
	windage::Algorithms::OpenCVRANSACestimator* estimator	= new windage::Algorithms::OpenCVRANSACestimator();
	windage::Algorithms::PoseRefiner* refiner			= new windage::Algorithms::PoseLMmethod();
	
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	searchtree->SetRatio(0.3);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	estimator->SetConfidence(0.90);
	estimator->SetMaxIteration(100);
	refiner->SetMaxIteration(20);

	tracking.AttatchCalibration(calibration);
	tracking.AttatchMatcher(searchtree);
	tracking.AttatchTracker(opticalflow);
	tracking.AttatchEstimator(estimator);
	tracking.AttatchRefiner(refiner);

	tracking.SetDitectionRatio(3);
	tracking.Initialize(WIDTH, HEIGHT);

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
	loader->DoLoad(FILE_NAME);

	int index = 0;
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = reconstructionPoints[i].GetFeatureList();

		// remove less matched feature points
		if(featureList->size() > 2)
		{
			windage::FeaturePoint feature = (*featureList)[0];
			windage::Vector4 point = reconstructionPoints[i].GetPoint();
			feature.SetPoint(windage::Vector3(point.x, point.y, point.z));
			feature.SetObjectID(0);
			feature.SetRepositoryID(index);

			// normalize descriptor
			int featureCount = (int)featureList->size();
			for(int j=1; j<featureCount; j++)
			{
				for(int k=0; k<(*featureList)[j].DESCRIPTOR_DIMENSION; k++)
					feature.descriptor[k] += (*featureList)[j].descriptor[k];
			}

			for(int k=0; k<feature.DESCRIPTOR_DIMENSION; k++)
				feature.descriptor[k] /= (double)featureCount;

			referenceRepository.push_back(feature);
			index++;
		}
	}
	std::cout << "reference repository size : " << referenceRepository.size() << std::endl;

	tracking.TrainingReference(&referenceRepository);
	trained = true;

	char message[100];
	bool saving = false;
	bool flip = true;
	bool processing = true;
	while(processing)
	{
		// capture image
		inputImage = cvRetrieveFrame(capture);
		if(flip)
			cvFlip(inputImage, inputImage);
		cvResize(inputImage, resizeImage);
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

		logger.updateTickCount();

		// track object
		if(trained)
		{
			tracking.UpdateCamerapose(grayImage);

			// draw result
			if(tracking.GetMatchingCount() > 9)
			{
//				tracking.DrawDebugInfo(resultImage);
				tracking.GetCameraParameter();
				calibration->DrawInfomation(resultImage, 100);

				//draw rectangle
//*
				float dx = 180.0f/2;
				float dy = 125.0f/2;
				float dz = 57.0f/2;

				CvPoint pt[8];
				pt[0] = calibration->ConvertWorld2Image(+dx, +dy, +dz);
				pt[1] = calibration->ConvertWorld2Image(+dx, -dy, +dz);
				pt[2] = calibration->ConvertWorld2Image(-dx, -dy, +dz);
				pt[3] = calibration->ConvertWorld2Image(-dx, +dy, +dz);
				pt[4] = calibration->ConvertWorld2Image(+dx, +dy, -dz);
				pt[5] = calibration->ConvertWorld2Image(+dx, -dy, -dz);
				pt[6] = calibration->ConvertWorld2Image(-dx, -dy, -dz);
				pt[7] = calibration->ConvertWorld2Image(-dx, +dy, -dz);

				for(int i=0; i<4; i++)
				{
					int i2 = i==3?0:i+1;
					cvLine(resultImage, pt[i], pt[i2], CV_RGB(0, 0, 0), 5);
					cvLine(resultImage, pt[i], pt[i2], CV_RGB(255, 0, 255), 3);
				}
				for(int i=4; i<8; i++)
				{
					int i2 = i==7?4:i+1;
					cvLine(resultImage, pt[i], pt[i2], CV_RGB(0, 0, 0), 5);
					cvLine(resultImage, pt[i], pt[i2], CV_RGB(255, 0, 255), 3);
				}
				for(int i=0; i<4; i++)
				{
					cvLine(resultImage, pt[i], pt[i+4], CV_RGB(0, 0, 0), 5);
					cvLine(resultImage, pt[i], pt[i+4], CV_RGB(255, 0, 255), 3);
				}
//*/
			}
		}
		matchingCount = tracking.GetMatchingCount();

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
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
