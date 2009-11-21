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

// adaptive threshold
#define ADAPTIVE_THRESHOLD

int fastThreshold = 70;
const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 30;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

const int OBJECT_COUNT = 5;
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double SCALE = 2.0;
const double REAL_WIDTH = 267.0 * SCALE;
const double REAL_HEIGHT = 200.0 * SCALE;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Vector3 GetMarkerTranslation(windage::Calibration* fromCalibration, windage::Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.x = cvGetReal2D(fromExtrinsicMatrix, 0, 3);
	fromTranslation.y = cvGetReal2D(fromExtrinsicMatrix, 1, 3);
	fromTranslation.z = cvGetReal2D(fromExtrinsicMatrix, 2, 3);

	windage::Vector3 toTranslation;
	toTranslation.x = cvGetReal2D(toExtrinsicMatrix, 0, 3);
	toTranslation.y = cvGetReal2D(toExtrinsicMatrix, 1, 3);
	toTranslation.z = cvGetReal2D(toExtrinsicMatrix, 2, 3);

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}

	fromRotation = fromRotation.Inverse();
	fromTranslation = fromRotation * fromTranslation;
	toTranslation = fromRotation * toTranslation;

	double x = toTranslation.x - fromTranslation.x;
	double y = toTranslation.y - fromTranslation.y;
	double z = toTranslation.z - fromTranslation.z;

	return windage::Vector3(x, y, z);
}

windage::Matrix3 GetMarkerRotation(windage::Calibration* fromCalibration, windage::Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}
	
	windage::Matrix3 toRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = cvGetReal2D(toExtrinsicMatrix, y, x);
		}
	}
	
	return fromRotation.Transpose() * toRotation;
}

windage::Matrix4 CalculateMarkerExtrinsicParameter(windage::Calibration* fromCalibration, windage::Matrix3 toRotation, windage::Vector3 toTranslation)
{
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.v[0] = cvGetReal2D(fromExtrinsicMatrix, 0, 3);
	fromTranslation.v[1] = cvGetReal2D(fromExtrinsicMatrix, 1, 3);
	fromTranslation.v[2] = cvGetReal2D(fromExtrinsicMatrix, 2, 3);

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}

	toTranslation = fromRotation * toTranslation;

	windage::Matrix3 rotation = fromRotation * toRotation;
	windage::Vector3 translation = fromTranslation + toTranslation;

	// set rotatino and translation
	windage::Matrix4 matrix;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			matrix.m[y][x] = rotation.m[y][x];
		}
		matrix.m[y][3] = translation.v[y];
	}
	matrix.m[3][0] = matrix.m[3][1] = matrix.m[3][2] = 0.0;
	matrix.m[3][3] = 1.0;

	return matrix;
}


void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	// saving
	bool saving = false;
	CvVideoWriter* writer = NULL;

	char message[100];
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* undistImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT*2), IPL_DEPTH_8U, 3);

	// Multipel tracker Initialize
	std::vector<IplImage*> trainingImage;
	std::vector<IplImage*> referenceImage;
	for(int i=1; i<=OBJECT_COUNT; i++)
	{
		sprintf(message, "reference%d_320.png", i);
		trainingImage.push_back(cvLoadImage(message, 0));
		sprintf(message, "reference%d.png", i);
		referenceImage.push_back(cvLoadImage(message));
	}

	windage::MultipleSURFTracker* multipleTracker = new windage::MultipleSURFTracker();
	multipleTracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	multipleTracker->InitializeOpticalFlow(WIDTH, HEIGHT, cvSize(8, 8), 3);
	multipleTracker->SetDetectIntervalTime(1.0);
	multipleTracker->SetPoseEstimationMethod(windage::POSE_3D);
	multipleTracker->SetOutlinerRemove(true);
	multipleTracker->SetFeatureExtractThreshold(30);
	for(int i=0; i<trainingImage.size(); i++)
	{
		std::cout << "attatch reference image #" << i << std::endl;
		multipleTracker->AttatchReferenceImage(trainingImage[i], REAL_WIDTH, REAL_HEIGHT, 8.0, 8);
	}

	// for undistortion and calculate coordinate
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	std::vector<windage::Matrix3> rotationList;
	rotationList.resize(trainingImage.size());
	std::vector<windage::Vector3> translationList;
	translationList.resize(trainingImage.size());

	std::vector<bool> foundList;
	foundList.resize(trainingImage.size());
	for(int i=0; i<foundList.size(); i++)
		foundList[i] = false;

	IplImage* grabFrame = NULL;

	bool updating = false;
	
	bool processing = true;
	cvNamedWindow("result");
	while(processing)
	{
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, undistImage);
		calibration->Undistortion( undistImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		multipleTracker->SetFeatureExtractThreshold(fastThreshold);
		multipleTracker->UpdateCameraPose(grayImage);

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();

		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		int featureCount = multipleTracker->GetFeatureCount();
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

		// draw tracking result
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			if(updating)
				foundList[i] = false;

			int matchedCount = multipleTracker->GetMatchedCount(i);
			if(matchedCount > FIND_FEATURE_COUNT)
			{
				multipleTracker->DrawOutLine(inputImage, i, true);
				multipleTracker->DrawInfomation(inputImage, i, 100.0 * SCALE);

				if(updating)
				{
					foundList[i] = true;
					rotationList[i] = GetMarkerRotation(multipleTracker->GetCameraParameter(0), multipleTracker->GetCameraParameter(i));
					translationList[i] = GetMarkerTranslation(multipleTracker->GetCameraParameter(0), multipleTracker->GetCameraParameter(i));
				}
			}
			
			if(foundList[i])
			{
				windage::Matrix3 rotation = rotationList[i];
				windage::Vector3 translation = translationList[i];

				windage::Matrix4 extrinsic = CalculateMarkerExtrinsicParameter(multipleTracker->GetCameraParameter(0), rotation, translation);
				calibration->SetExtrinsicMatrix(extrinsic.m1);

				// draw outline
				windage::Vector3 temp = windage::Vector3(0.0, 0.0, 0.0);
				CvScalar color = CV_RGB(255, 255, 0);
				cvLine(inputImage,  calibration->ConvertWorld2Image(temp.x-REAL_WIDTH/2.0, temp.y-REAL_HEIGHT/2.0, temp.z+0.0), 
									calibration->ConvertWorld2Image(temp.x+REAL_WIDTH/2.0, temp.y-REAL_HEIGHT/2.0, temp.z+0.0), color, 5);
				cvLine(inputImage,  calibration->ConvertWorld2Image(temp.x+REAL_WIDTH/2.0, temp.y-REAL_HEIGHT/2.0, temp.z+0.0), 
									calibration->ConvertWorld2Image(temp.x+REAL_WIDTH/2.0, temp.y+REAL_HEIGHT/2.0, temp.z+0.0), color, 5);
				cvLine(inputImage,  calibration->ConvertWorld2Image(temp.x+REAL_WIDTH/2.0, temp.y+REAL_HEIGHT/2.0, temp.z+0.0), 
									calibration->ConvertWorld2Image(temp.x-REAL_WIDTH/2.0, temp.y+REAL_HEIGHT/2.0, temp.z+0.0), color, 5);
				cvLine(inputImage,  calibration->ConvertWorld2Image(temp.x-REAL_WIDTH/2.0, temp.y+REAL_HEIGHT/2.0, temp.z+0.0), 
									calibration->ConvertWorld2Image(temp.x-REAL_WIDTH/2.0, temp.y-REAL_HEIGHT/2.0, temp.z+0.0), color, 5);

				// draw axis
				CvPoint center = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z);
				CvPoint centerX = calibration->ConvertWorld2Image(temp.x + 50.0 * SCALE, temp.y, temp.z);
				CvPoint centerY = calibration->ConvertWorld2Image(temp.x, temp.y + 50.0 * SCALE, temp.z);
				CvPoint centerZ = calibration->ConvertWorld2Image(temp.x, temp.y, temp.z + 50.0 * SCALE);

				cvLine(inputImage, center, centerX, CV_RGB(255, 0, 0), 5);
				cvLine(inputImage, center, centerY, CV_RGB(0, 255, 0), 5);
				cvLine(inputImage, center, centerZ, CV_RGB(0, 0, 255), 5);

				// draw data
				center.x += 10;
				center.y += 30;
				sprintf(message, "Reference #%d T:(%.1lf, %.1lf, %.1lf)", i, translation.x, translation.y, translation.z);
				windage::Utils::DrawTextToImage(inputImage, center, message);

				windage::Vector3 euler = windage::Quaternion::DcmToEuler(rotation);
				center.y += 20;
				sprintf(message, "              R:(%.1lf, %.1lf, %.1lf)", euler.x*180.0/CV_PI, euler.y*180.0/CV_PI, euler.z*180.0/CV_PI);
				windage::Utils::DrawTextToImage(inputImage, center, message);
			}
		}
		updating = false;

		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count ");
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
		for(int i=0; i<trainingImage.size(); i++)
		{
			sprintf(message, "#%d:%d ", i, multipleTracker->GetMatchedCount(i));
			windage::Utils::DrawTextToImage(inputImage, cvPoint(160 + 65*i, 70), message);
		}


/*
		const int PIP_RATIO = 6;
		const int PIP_WIDTH = WIDTH/PIP_RATIO;
		const int PIP_HEIGHT = HEIGHT/PIP_RATIO * 2;

		cvZero(resultImage);
		for(int i=0; i<trainingImage.size(); i++)
		{
			cvSetImageROI(tempImage2, cvRect(0, 0, WIDTH, HEIGHT));
			cvCopy(referenceImage[i], tempImage2);
			cvSetImageROI(tempImage2, cvRect(0, HEIGHT, WIDTH, HEIGHT));
			cvCopy(inputImage, tempImage2);
			cvResetImageROI(tempImage2);
			multipleTracker->DrawDebugInfo2(tempImage2, i);

			CvRect rect = cvRect(10 + PIP_WIDTH * i, HEIGHT - 10 - PIP_HEIGHT, PIP_WIDTH, PIP_HEIGHT);
			cvRectangle(resultImage, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width, rect.y+rect.height), CV_RGB(255, 255, 255), 5);
			cvSetImageROI(resultImage, rect);
			cvResize(tempImage2, resultImage);
			cvResetImageROI(resultImage);

//			cvNamedWindow("temp");
//			if(multipleTracker->GetMatchedCount(i) > FIND_FEATURE_COUNT && i==0)
//				cvShowImage("temp", tempImage2);
		}
//*/
//		windage::Utils::CompundImmersiveImage(resultImage, inputImage, CV_RGB(0, 0, 0), 0.70);
//*/

		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'u':
		case 'U':
			updating = true;
			break;
		case 's':
			saving = true;
			if(writer) cvReleaseVideoWriter(&writer);
			writer = cvCreateVideoWriter("saveimage\\capture.avi", CV_FOURCC_DEFAULT, 30, cvSize(inputImage->width, inputImage->height), 1);
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
	}

	if(writer) cvReleaseVideoWriter(&writer);
	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
