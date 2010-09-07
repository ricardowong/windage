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
#include <windows.h>
#include <process.h>

#pragma comment(lib, "WS2_32.lib")
#pragma comment(lib, "winmm.lib")

#include <cv.h>
#include <highgui.h>

#include <windage.h>

// communication
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"

const int PORT = 7239;

// for object tracking
const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int FEATURE_COUNT = WIDTH;

const double SCALE_FACTOR = 2.0;
const int SCALE_STEP = 3;
const double REPROJECTION_ERROR = 5.0;

#define USE_ADAPTIVE_THRESHOLD 1
#define USE_TEMPLATE_IMAEG 1
const char* TEMPLATE_IMAGE = "reference1_320.png";
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

// communication
class RelationPacketListener : public osc::OscPacketListener
{
private:
	windage::Matrix3 rotation;
	windage::Vector3 translation;
public:
	windage::Matrix3 GetRotation(){return this->rotation;}
	windage::Vector3 GetTranslation(){return this->translation;}
	
protected:
    virtual void ProcessMessage( const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint )
    {
        try
		{
            if( strcmp( m.AddressPattern(), "MultimarkerRelation" ) == 0 )
			{
				osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
				args >> rotation.m[0][0] >> rotation.m[0][1] >> rotation.m[0][2]
					>> rotation.m[1][0] >> rotation.m[1][1] >> rotation.m[1][2]
					>> rotation.m[2][0] >> rotation.m[2][1] >> rotation.m[2][2]
					>> translation.x >> translation.y >> translation.z >> osc::EndMessage;
				std::cout << "recevie data" << std::endl;
            }
        }
		catch( osc::Exception& e )
		{
            // any parsing errors such as unexpected argument types, or 
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: " << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};

unsigned int WINAPI SystemListenerThread(void* pArg)
{
	RelationPacketListener* listener = (RelationPacketListener*)pArg;

	UdpListeningReceiveSocket receiveSocket(IpEndpointName(IpEndpointName::ANY_ADDRESS, PORT), listener);
	receiveSocket.RunUntilSigInt();
	return 0;
}

void main()
{
	// communication
	RelationPacketListener relationListener;
	HANDLE nHandle = (HANDLE)_beginthreadex(NULL, 0, SystemListenerThread, (void*)&relationListener, 0, NULL);

	// tracking
	windage::Logger logger(&std::cout);

	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	windage::Calibration relationCalibration;
	relationCalibration.Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	// create and initialize tracker
	windage::Frameworks::PlanarObjectTracking tracking;
	windage::Calibration* calibration;
	windage::Algorithms::FeatureDetector* detector;
	windage::Algorithms::SearchTree* searchtree;
	windage::Algorithms::OpticalFlow* opticalflow;
	windage::Algorithms::HomographyEstimator* estimator;
	windage::Algorithms::OutlierChecker* checker;
	windage::Algorithms::HomographyRefiner* refiner;
	windage::Algorithms::KalmanFilter* filter;

	calibration = new windage::Calibration();
	detector = new windage::Algorithms::SIFTGPUdetector();
	searchtree = new windage::Algorithms::FLANNtree();
	opticalflow = new windage::Algorithms::OpticalFlow();
	estimator = new windage::Algorithms::ProSACestimator();
	checker = new windage::Algorithms::OutlierChecker();
	refiner = new windage::Algorithms::LMmethod();
	filter = new windage::Algorithms::KalmanFilter();

	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.5);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(15, 15), 3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	checker->SetReprojectionError(REPROJECTION_ERROR * 3);
	refiner->SetMaxIteration(10);

	tracking.AttatchCalibration(calibration);
	tracking.AttatchDetetor(detector);
	tracking.AttatchMatcher(searchtree);
	tracking.AttatchTracker(opticalflow);
	tracking.AttatchEstimator(estimator);
	tracking.AttatchChecker(checker);
	tracking.AttatchRefiner(refiner);
//	tracking.AttatchFilter(filter);

	tracking.SetDitectionRatio(10);
	tracking.Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	int keypointCount = 0;
	int matchingCount = 0;
	double threshold = 50.0;
	double processingTime = 0.0;

	bool trained = false;

#if USE_TEMPLATE_IMAEG
	IplImage* sampleImage = cvLoadImage(TEMPLATE_IMAGE, 0);
	detector->SetThreshold(30.0);
	tracking.AttatchReferenceImage(sampleImage);
	tracking.TrainingReference(SCALE_FACTOR, SCALE_STEP);
	detector->SetThreshold(threshold);
	trained = true;
#endif

	char message[100];
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

			// adaptive threshold
#if USE_ADAPTIVE_THRESHOLD
			int localcount = detector->GetKeypointsCount();
			if(keypointCount != localcount)
			{
				if(localcount > FEATURE_COUNT)
					threshold += 1;
				if(localcount < FEATURE_COUNT)
					threshold -= 1;
				detector->SetThreshold(threshold);
				keypointCount = localcount;
			}
#endif
			// draw result
//			detector->DrawKeypoints(resultImage);
			if(tracking.GetMatchingCount() > 10)
			{
//				tracking.DrawDebugInfo(resultImage);
				tracking.DrawOutLine(resultImage, true);
				calibration->DrawInfomation(resultImage, 100);

				// calculate relation
				windage::Matrix4 extrinsic = windage::Coordinator::MultiMarkerCoordinator::CalculateExtrinsic(calibration, relationListener.GetRotation(), relationListener.GetTranslation());
				relationCalibration.SetExtrinsicMatrix(extrinsic.m1);
				relationCalibration.DrawInfomation(resultImage, 100);
			}
		}
		matchingCount = tracking.GetMatchingCount();

		processingTime = logger.calculateProcessTime();
		logger.log("processingTime", processingTime);
		logger.logNewLine();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Feature Count : %d, Threshold : %.0lf", keypointCount, threshold);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), 0.6, message);
		sprintf_s(message, "Matching Count : %d", matchingCount);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), 0.6, message);

		sprintf_s(message, "Press 'Space' to track the current image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-10), 0.5, message);
		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(WIDTH-270, HEIGHT-25), 0.5, message);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		case 'f':
		case 'F':
			flip = !flip;
			break;
		case ' ':
		case 's':
		case 'S':
			detector->SetThreshold(30.0);
			tracking.AttatchReferenceImage(grayImage);
			tracking.TrainingReference(SCALE_FACTOR, SCALE_STEP);
			detector->SetThreshold(threshold);
			trained = true;
			break;
		}		
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
