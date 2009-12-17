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

#define ADAPTIVE_THRESHOLD

const int OBJECT_COUNT = 6;
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double CUBE_SIZE = 1000.0;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

// check cube plane
//   1
// 2 3 4 5
//   6
windage::Matrix3 GetRotation(int index)
{
	windage::Matrix3 rotation;
	rotation.m[0][0] = 1.0;
	rotation.m[0][1] = 0.0;
	rotation.m[0][2] = 0.0;

	rotation.m[1][0] = 0.0;
	rotation.m[1][1] = 1.0;
	rotation.m[1][2] = 0.0;

	rotation.m[2][0] = 0.0;
	rotation.m[2][1] = 0.0;
	rotation.m[2][2] = 1.0;

	double orthoRadian = 90 * CV_PI/180.0;
	switch(index)
	{
	case 1: // x-axis
		rotation.m[1][1] = cos(-orthoRadian);
		rotation.m[1][2] = sin(-orthoRadian);

		rotation.m[2][1] = -sin(-orthoRadian);
		rotation.m[2][2] = cos(-orthoRadian);
		break;
	case 2: // y-axis
		rotation.m[0][0] = cos(-orthoRadian);
		rotation.m[0][2] = -sin(-orthoRadian);

		rotation.m[2][0] = sin(-orthoRadian);
		rotation.m[2][2] = cos(-orthoRadian);
		break;
	case 3:
		// main oriantation
		break;
	case 4: // y-axis
		rotation.m[0][0] = cos(orthoRadian);
		rotation.m[0][2] = -sin(orthoRadian);

		rotation.m[2][0] = sin(orthoRadian);
		rotation.m[2][2] = cos(orthoRadian);
		break;
	case 5: // y-axis
		rotation.m[0][0] = cos(orthoRadian*2);
		rotation.m[0][2] = -sin(orthoRadian*2);

		rotation.m[2][0] = sin(orthoRadian*2);
		rotation.m[2][2] = cos(orthoRadian*2);
		break;
	case 6: // x-axis
		rotation.m[1][1] = cos(-orthoRadian);
		rotation.m[1][2] = -sin(-orthoRadian);

		rotation.m[2][1] = sin(-orthoRadian);
		rotation.m[2][2] = cos(-orthoRadian);
		break;
	}

	return rotation;
}

//   1
// 2 3 4 5
//   6
windage::Vector3 GetTranslation(int index)
{
	windage::Vector3 translation;
	translation.x = 0.0;
	translation.y = 0.0;
	translation.z = 0.0;

	switch(index)
	{
	case 1:
		translation.z = -CUBE_SIZE/2.0;
		break;
	case 2:
		translation.z = -CUBE_SIZE/2.0;
		break;
	case 3: // main
		translation.z = -CUBE_SIZE/2.0;
		break;
	case 4:
		translation.z = -CUBE_SIZE/2.0;
		break;
	case 5:
		translation.z = -CUBE_SIZE/2.0;
		break;
	case 6:
		translation.z = -CUBE_SIZE/2.0;
		break;
	}

	return translation;
}

windage::Vector3 GetMarkerRotation(windage::Calibration* calibration)
{
	CvMat* toExtrinsicMatrix = calibration->GetExtrinsicMatrix();
	
	windage::Matrix3 toRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = cvGetReal2D(toExtrinsicMatrix, y, x);
		}
	}
	return windage::Quaternion::DcmToEuler(toRotation);
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

double CalcReprojectionArea(windage::Calibration* cameraParameter)
{
	double width = CUBE_SIZE/2.0;
	double height = CUBE_SIZE/2.0;

	CvPoint point1 = cameraParameter->ConvertWorld2Image(-width, -height, 0.0);
	CvPoint point2 = cameraParameter->ConvertWorld2Image(+width, -height, 0.0);
	CvPoint point3 = cameraParameter->ConvertWorld2Image(+width, +height, 0.0);
	CvPoint point4 = cameraParameter->ConvertWorld2Image(-width, +height, 0.0);

	CvScalar cameraPoint = cameraParameter->GetCameraPosition();
	double distance =	cameraPoint.val[0]*cameraPoint.val[0] + 
						cameraPoint.val[1]*cameraPoint.val[1] + 
						cameraPoint.val[2]*cameraPoint.val[2];

	double area1 = abs((point2.x - point1.x) * (point4.y - point1.y) - (point2.y - point1.y) * (point4.x - point1.x)) / 2.0;
	double area2 = abs((point2.x - point3.x) * (point4.y - point3.y) - (point2.y - point3.y) * (point4.x - point3.x)) / 2.0;

	// scale (about) : 0.0 ~ 10.0
	return ((area1 + area2) * (distance)) / 100000000000.0;
}

double Scoring(double area, int matchingCount)
{
	if(matchingCount < FIND_FEATURE_COUNT)
		return 0.0;
	return area + ((double)matchingCount/10.0);
}

void DrawOutLine(windage::Calibration* cameraParameter, IplImage* colorImage, bool drawCross)
{
	int r = 255;
	int g = 0;
	int b = 0;

	double width = CUBE_SIZE/2.0;
	double height = CUBE_SIZE/2.0;
	double depth = CUBE_SIZE/2.0;

	int size = 4;

	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(255, 255, 255);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, +depth),	cameraParameter->ConvertWorld2Image(+width, -height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, +depth),	cameraParameter->ConvertWorld2Image(+width, +height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, +depth),	cameraParameter->ConvertWorld2Image(-width, +height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, +depth),	cameraParameter->ConvertWorld2Image(-width, -height, +depth),	color2, 6);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, -height, -depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, +height, -depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, +height, -depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, -height, -depth),	color2, 6);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, -depth),	cameraParameter->ConvertWorld2Image(-width, -height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, -height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, -depth),	cameraParameter->ConvertWorld2Image(+width, +height, +depth),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, +height, +depth),	color2, 6);


	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, +depth),	cameraParameter->ConvertWorld2Image(+width, -height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, +depth),	cameraParameter->ConvertWorld2Image(+width, +height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, +depth),	cameraParameter->ConvertWorld2Image(-width, +height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, +depth),	cameraParameter->ConvertWorld2Image(-width, -height, +depth),	color, 2);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, -height, -depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, +height, -depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, +height, -depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, -height, -depth),	color, 2);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, -height, -depth),	cameraParameter->ConvertWorld2Image(-width, -height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, -height, -depth),	cameraParameter->ConvertWorld2Image(+width, -height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+width, +height, -depth),	cameraParameter->ConvertWorld2Image(+width, +height, +depth),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-width, +height, -depth),	cameraParameter->ConvertWorld2Image(-width, +height, +depth),	color, 2);
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

	// Multipel tracker Initialize
	std::vector<IplImage*> trainingImage;
	for(int i=1; i<=OBJECT_COUNT; i++)
	{
		sprintf(message, "cube/reference%d.png", i);
		trainingImage.push_back(cvLoadImage(message, 0));
	}

	windage::MultipleSURFTracker* multipleTracker = new windage::MultipleSURFTracker();
	multipleTracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	multipleTracker->InitializeOpticalFlow(WIDTH, HEIGHT, cvSize(8, 8), 3);
	multipleTracker->SetDetectIntervalTime(1.0/1.0);
	multipleTracker->SetPoseEstimationMethod(windage::RANSAC);
	multipleTracker->SetOutlinerRemove(true);
	multipleTracker->SetRefinement(true);
	multipleTracker->SetPosePointCount(FIND_FEATURE_COUNT);
	multipleTracker->SetFeatureExtractThreshold(30);
	for(int i=0; i<trainingImage.size(); i++)
	{
		std::cout << "attatch reference image #" << i << std::endl;
		multipleTracker->AttatchReferenceImage(trainingImage[i], CUBE_SIZE, CUBE_SIZE, 4.0, 8);
	}

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 1000;
	const int THRESHOLD_STEP = 1;

	IplImage* grabFrame = NULL;
	
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
		// find max matched plane
		std::vector<int> matcingCountList; matcingCountList.resize(multipleTracker->GetTrackerCount());
		int maxScoreIndex = -1;
		double maxScore = 0.0;
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			double area = CalcReprojectionArea(multipleTracker->GetCameraParameter(i));
			int matchedCount = multipleTracker->GetMatchedCount(i);
			matcingCountList[i] = matchedCount;

			double score = Scoring(area, matchedCount);
			if(score > maxScore)
			{
				maxScore = area;
				maxScoreIndex = i;
			}

			// delete tracking points when too small space
			if(area < 2.0)
			{
				multipleTracker->DeleteTrackingPoints(i);
			}

//			std::cout << area << " : " << matchedCount << " : " << score << std::endl;
		}

		// draw tracking result
		windage::Vector3 eulerRotation;
		if(maxScoreIndex >= 0)
		if(matcingCountList[maxScoreIndex] > FIND_FEATURE_COUNT)
		{
			int i = maxScoreIndex;
			windage::Matrix4 extrinsic = CalculateMarkerExtrinsicParameter(multipleTracker->GetCameraParameter(i), GetRotation(i+1), GetTranslation(i+1));
			calibration->SetExtrinsicMatrix(extrinsic.m1);

			DrawOutLine(calibration, inputImage, false);
			calibration->DrawInfomation(inputImage, CUBE_SIZE);

			CvPoint center = multipleTracker->GetCameraParameter(i)->ConvertWorld2Image(0.0, 0.0, 0.0);
			
			center.x += 10;
			center.y += 10;
			sprintf(message, "Reference #%d", i);
			windage::Utils::DrawTextToImage(inputImage, center, message);

			eulerRotation = GetMarkerRotation(calibration);

//			multipleTracker->DrawDebugInfo(inputImage, maxScoreIndex);
		}

		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count ");
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
		for(int i=0; i<OBJECT_COUNT; i++)
		{
			sprintf(message, "#%d:%d ", i, multipleTracker->GetMatchedCount(i));
			windage::Utils::DrawTextToImage(inputImage, cvPoint(160 + 65*i, 70), message);
		}

		eulerRotation *= 180.0/CV_PI;
		sprintf(message, "Rotation : %.2lf, %.2lf, %.2lf", eulerRotation.x, eulerRotation.y, eulerRotation.z);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 90), message); 


		if(saving)
		{
			if(writer) cvWriteFrame(writer, inputImage);
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 's':
		case 'S':
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
