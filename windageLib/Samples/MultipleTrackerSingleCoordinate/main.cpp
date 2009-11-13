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

const int OBJECT_COUNT = 5;
const int FIND_FEATURE_COUNT = 10;

const int WIDTH = 640;
const int HEIGHT = 480;

const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Vector3 DcmToEuler(windage::Matrix3 dcm)
{
	windage::Vector3 eular;

	eular.x = atan2(dcm.m[2][0], dcm.m[2][1]);
	eular.y = acos(dcm.m[2][2]);
	eular.z = -atan2(dcm.m[0][2], dcm.m[1][2]);
	return eular;
}

windage::Vector4 EulerToQuaternion(windage::Vector3 euler)
{
	windage::Vector4 quaternion;

	quaternion.x = -cos((euler.x - euler.z)/2.0) * sin(euler.y/2.0);
	quaternion.y = sin((euler.x-euler.z)/2.0) * sin(euler.y/2.0);
	quaternion.z = -sin((euler.x + euler.z)/2.0) * cos(euler.y/2);
	quaternion.w = cos((euler.x + euler.z)/2.0) * cos(euler.y/2);
	return quaternion;
}

windage::Vector3 EulerToQuaternion(windage::Vector4 quaternion)
{
	windage::Vector3 euler;

	euler.x = atan2((quaternion.x*quaternion.z) + (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) - (quaternion.x*quaternion.w));
	euler.y = acos(-(quaternion.x*quaternion.x)-(quaternion.y*quaternion.y)+(quaternion.z*quaternion.z)+(quaternion.w*quaternion.w));
	euler.z = -atan2((quaternion.x*quaternion.z) - (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) + (quaternion.x*quaternion.w));
	return euler;
}

windage::Matrix3 QuaternionToDcm(windage::Vector4 quaternion)
{
	windage::Matrix3 dcm;

	dcm.m[0][0] = 1.0 - 2.0*quaternion.y*quaternion.y - 2.0*quaternion.z*quaternion.z;
	dcm.m[0][1] = 2.0*(quaternion.x*quaternion.y - quaternion.z*quaternion.w);
	dcm.m[0][2] = 2.0*(quaternion.x*quaternion.z + quaternion.y*quaternion.w);

	dcm.m[1][0] = 2.0*(quaternion.x*quaternion.y + quaternion.z*quaternion.w);
	dcm.m[1][1] = 1.0 - 2.0*quaternion.x*quaternion.x - 2.0*quaternion.z*quaternion.z;
	dcm.m[1][2] = 2.0*(quaternion.y*quaternion.z - quaternion.x*quaternion.w);

	dcm.m[2][0] = 2.0*(quaternion.x*quaternion.z - quaternion.y*quaternion.w);
	dcm.m[2][1] = 2.0*(quaternion.x*quaternion.w + quaternion.y*quaternion.z);
	dcm.m[2][2] = 1.0 - 2.0*quaternion.z*quaternion.z - 2.0*quaternion.z*quaternion.z;
	return dcm;
}

windage::Vector4 DcmToQuaternion(windage::Matrix3 dcm)
{
	windage::Vector4 quaternion;

	quaternion.w = sqrt(1.0 + dcm.m[0][0] + dcm.m[1][1] + dcm.m[2][2]) / 2.0;
	quaternion.x = (dcm.m[0][1] - dcm.m[2][1]) / (4.0 * quaternion.w);
	quaternion.y = (dcm.m[2][0] - dcm.m[0][3]) / (4.0 * quaternion.w);
	quaternion.z = (dcm.m[0][1] - dcm.m[1][0]) / (4.0 * quaternion.w);
	return quaternion;
}


windage::Vector3 GetTranslation(windage::Calibration* toCalibration, windage::Calibration* fromCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	CvMat* fromTranslationVectorFinal = cvCreateMat(3, 1, CV_64FC1);
	CvMat* fromTranslationVector = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(fromTranslationVector, 0, cvGetReal2D(fromExtrinsicMatrix, 0, 3));
	cvSetReal1D(fromTranslationVector, 1, cvGetReal2D(fromExtrinsicMatrix, 1, 3));
	cvSetReal1D(fromTranslationVector, 2, cvGetReal2D(fromExtrinsicMatrix, 2, 3));

	CvMat* toTranslationVectorFinal = cvCreateMat(3, 1, CV_64FC1);
	CvMat* toTranslationVector = cvCreateMat(3, 1, CV_64FC1);
	cvSetReal1D(toTranslationVector, 0, cvGetReal2D(toExtrinsicMatrix, 0, 3));
	cvSetReal1D(toTranslationVector, 1, cvGetReal2D(toExtrinsicMatrix, 1, 3));
	cvSetReal1D(toTranslationVector, 2, cvGetReal2D(toExtrinsicMatrix, 2, 3));

	CvMat* toRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			cvSetReal2D(toRotationMatrix, x, y, cvGetReal2D(toExtrinsicMatrix, x, y));
		}
	}
	CvMat* toRotationMatrixInvers = cvCreateMat(3, 3, CV_64FC1);
	cvInvert(toRotationMatrix, toRotationMatrixInvers);
	cvMatMul(toRotationMatrixInvers, toTranslationVector, toTranslationVectorFinal);
	cvMatMul(toRotationMatrixInvers, fromTranslationVector, fromTranslationVectorFinal);


	double x = cvGetReal1D(fromTranslationVectorFinal, 0) - cvGetReal1D(toTranslationVectorFinal, 0);
	double y = cvGetReal1D(fromTranslationVectorFinal, 1) - cvGetReal1D(toTranslationVectorFinal, 1);
	double z = cvGetReal1D(fromTranslationVectorFinal, 2) - cvGetReal1D(toTranslationVectorFinal, 2);

	cvReleaseMat(&fromTranslationVectorFinal);
	cvReleaseMat(&fromTranslationVector);
	cvReleaseMat(&toTranslationVectorFinal);
	cvReleaseMat(&toTranslationVector);
	cvReleaseMat(&toRotationMatrix);
	cvReleaseMat(&toRotationMatrixInvers);

	return windage::Vector3(x, y, z);
}

windage::Vector3 GetRotation(windage::Calibration* toCalibration, windage::Calibration* fromCalibration)
{
	windage::Matrix3 toRotation;
	windage::Matrix3 fromRotation;

	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = fromCalibration->GetExtrinsicMatrix();

	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = cvGetReal2D(fromExtrinsicMatrix, y, x);
		}
	}

	windage::Vector3 fromEuler = DcmToEuler(fromRotation);

	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = cvGetReal2D(toExtrinsicMatrix, y, x);
		}
	}
	windage::Vector3 toEuler = DcmToEuler(toRotation);

	return fromEuler - toEuler;
}

windage::Matrix3 ConvertQuaternionMatrix(windage::Vector3 euclidean)
{
	windage::Matrix3 rotation;

//	euclidean = -euclidean;
	euclidean.x = -euclidean.y;
	euclidean.x = 0;
//	euclidean.y = 0;
//	euclidean.z = 0;

	double sinX = sin(euclidean.x);
	double sinY = sin(euclidean.y);
	double sinZ = sin(euclidean.z);
	double cosX = cos(euclidean.x);
	double cosY = cos(euclidean.y);
	double cosZ = cos(euclidean.z);

	rotation.m[0][0] = cosY*cosZ;
	rotation.m[0][1] = (-cosX)*sinZ + sinX*sinY*cosZ ;
	rotation.m[0][2] = sinX*sinZ + cosX*sinY*cosZ;

	rotation.m[1][0] = cosY*sinZ;
	rotation.m[1][1] = cosX*cosZ+sinX*sinY*sinZ;
	rotation.m[1][2] = (-sinX)*cosZ + cosX*sinY*sinZ;

	rotation.m[2][0] = -sinY;
	rotation.m[2][1] = sinX*cosY;
	rotation.m[2][2] = cosX*cosY;

	return rotation;
}




void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

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
	multipleTracker->SetPoseEstimationMethod(windage::PROSAC);
	multipleTracker->SetOutlinerRemove(true);
	multipleTracker->SetFeatureExtractThreshold(30);
	for(int i=0; i<trainingImage.size(); i++)
	{
		std::cout << "attatch reference image #" << i << std::endl;
		multipleTracker->AttatchReferenceImage(trainingImage[i], 26.70, 20.00, 4.0, 8);
	}

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 70;
	const int MAX_FAST_THRESHOLD = 80;
	const int MIN_FAST_THRESHOLD = 40;
	const int ADAPTIVE_THRESHOLD_VALUE = 500;
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

		multipleTracker->DrawOutLine(inputImage, 0, true);
		multipleTracker->DrawInfomation(inputImage, 0, 5.0);

		// draw tracking result
		for(int i=0; i<multipleTracker->GetTrackerCount(); i++)
		{
			int matchedCount = multipleTracker->GetMatchedCount(i);
			if(matchedCount > FIND_FEATURE_COUNT)
			{
				multipleTracker->DrawOutLine(inputImage, i, true);
				multipleTracker->DrawInfomation(inputImage, i, 5.0);

				windage::Vector3 translation = GetTranslation(multipleTracker->GetCameraParameter(0), multipleTracker->GetCameraParameter(i));
				windage::Vector3 rotation = GetRotation(multipleTracker->GetCameraParameter(0), multipleTracker->GetCameraParameter(i));
				windage::Matrix3 quaternion = QuaternionToDcm(EulerToQuaternion(rotation));

				windage::Vector3 axisX = quaternion * windage::Vector3(5.0, 0.0, 0.0);
				windage::Vector3 axisY = quaternion * windage::Vector3(0.0, 5.0, 0.0);
				windage::Vector3 axisZ = quaternion * windage::Vector3(0.0, 0.0, 5.0);

				CvPoint center = multipleTracker->GetCameraParameter(0)->ConvertWorld2Image(translation.x, translation.y, translation.z);
				
				CvPoint centerP = multipleTracker->GetCameraParameter(0)->ConvertWorld2Image(0.0, 0.0, 0.0);
				CvPoint centerX = multipleTracker->GetCameraParameter(0)->ConvertWorld2Image(axisX.x, axisX.y, axisX.z);
				CvPoint centerY = multipleTracker->GetCameraParameter(0)->ConvertWorld2Image(axisY.x, axisY.y, axisY.z);
				CvPoint centerZ = multipleTracker->GetCameraParameter(0)->ConvertWorld2Image(axisZ.x, axisZ.y, axisZ.z);

				cvLine(inputImage, centerP, centerX, CV_RGB(255, 0, 0), 5);
				cvLine(inputImage, centerP, centerY, CV_RGB(0, 255, 0), 5);
				cvLine(inputImage, centerP, centerZ, CV_RGB(0, 0, 255), 5);
				
				center.x += 10;
				center.y += 10;
				sprintf(message, "Reference #%d (%.1lf, %.1lf, %.1lf)", i, translation.x, translation.y, translation.z);
				windage::Utils::DrawTextToImage(inputImage, center, message);

				center.y += 20;
				sprintf(message, "              (%.1lf, %.1lf, %.1lf)", rotation.x*180.0/CV_PI, rotation.y*180.0/CV_PI, rotation.z*180.0/CV_PI);
				windage::Utils::DrawTextToImage(inputImage, center, message);
			}

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


//*
		const int PIP_RATIO = 6;
		const int PIP_WIDTH = WIDTH/PIP_RATIO;
		const int PIP_HEIGHT = HEIGHT/PIP_RATIO * 2;

		cvZero(resultImage);
		for(int i=0; i<OBJECT_COUNT; i++)
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
//			if(multipleTracker->GetMatchedCount(i) > FIND_FEATURE_COUNT)
//				cvShowImage("temp", tempImage2);
		}
//*/
		windage::Utils::CompundImmersiveImage(resultImage, inputImage, CV_RGB(0, 0, 0), 0.30);
//*/

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
