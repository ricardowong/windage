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

#include <cv.h>
#include <highgui.h>

#include "windageTest.h"

#include "windage.h"
#include "Reconstruction/StereoReconstruction.h"

class StereoReconstructionTest : public windageTest
{
private:
	IplImage* inputImage1;
	IplImage* inputImage2;
	IplImage* grayImage1;
	IplImage* grayImage2;
	CvSize imageSize;

	windage::Calibration* calibration1;
	windage::Calibration* calibration2;
	windage::Algorithms::FeatureDetector* detectorRef;
	windage::Algorithms::FeatureDetector* detectorSce;
	std::vector<windage::FeaturePoint> refPoints;
	std::vector<windage::FeaturePoint> scePoints;
	windage::Algorithms::SearchTree* matcher;

public:
	StereoReconstructionTest() : windageTest("StereoReconstruction Test", "StereoReconstruction")
	{
		inputImage1 = NULL;
		inputImage2 = NULL;
		grayImage1 = NULL;
		grayImage2 = NULL;

		detectorRef = NULL;
		detectorSce = NULL;
		matcher = NULL;

		this->Do();
	}
	~StereoReconstructionTest()
	{
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;

		if(calibration1) delete calibration1;
		calibration1 = NULL;
		if(calibration2) delete calibration2;
		calibration2 = NULL;
		if(detectorRef) delete detectorRef;
		detectorRef = NULL;
		if(detectorSce) delete detectorSce;
		detectorSce = NULL;
		if(matcher) delete matcher;
		matcher = NULL;
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters

		// load reference image
		inputImage1 = cvLoadImage(REFERENCE_IMAGE_FILENAME.c_str());
		inputImage2 = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		imageSize = cvGetSize(inputImage1);

		grayImage1 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		grayImage2 = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
		resultImage = cvCreateImage(cvSize(imageSize.width*2, imageSize.height), IPL_DEPTH_8U, 3);

		cvSetImageROI(resultImage, cvRect(0, 0, imageSize.width, imageSize.height));
		cvCopyImage(inputImage1, resultImage);
		cvSetImageROI(resultImage, cvRect(imageSize.width, 0, imageSize.width, imageSize.height));
		cvCopyImage(inputImage2, resultImage);
		cvResetImageROI(resultImage);

		cvCvtColor(inputImage1, grayImage1, CV_BGR2GRAY);
		cvCvtColor(inputImage2, grayImage2, CV_BGR2GRAY);

		calibration1 = new windage::Calibration();
		calibration2 = new windage::Calibration();
		detectorRef = new windage::Algorithms::WSURFdetector();
		detectorSce = new windage::Algorithms::WSURFdetector();
		matcher = new windage::Algorithms::FLANNtree();

		detectorRef->DoExtractKeypointsDescriptor(grayImage1);
		detectorSce->DoExtractKeypointsDescriptor(grayImage2);

		matcher->SetRatio(0.7);
		matcher->Training(detectorRef->GetKeypoints());

		for(unsigned int i=0; i<detectorSce->GetKeypoints()->size(); i++)
		{
			int index = matcher->Matching((*detectorSce->GetKeypoints())[i]);
			if(index >= 0)
			{
				windage::FeaturePoint fp1 = (*detectorRef->GetKeypoints())[index];
				windage::FeaturePoint fp2 = (*detectorSce->GetKeypoints())[i];

				refPoints.push_back(fp1);
				scePoints.push_back(fp2);
			}
		}

		calibration1->Initialize(1200, 1200, 200, 160, 0, 0, 0, 0);
		calibration2->Initialize(1200, 1200, 200, 160, 0, 0, 0, 0);

		return true;
	}

	bool TestMemoryRelease(std::string* message)
	{
		// checek the memory leak
		const int size = 10;
		char memoryAddress1[size];
		char memoryAddress2[size];

		void* p1 = 0;
		void* p2 = 0;
		int compair = 0;
		double error = 0.0;

		windage::Reconstruction::StereoReconstruction* reconstruction1 = new windage::Reconstruction::StereoReconstruction();
		p1 = (void*)reconstruction1;
		reconstruction1->AttatchBaseCameraParameter(this->calibration1);
		reconstruction1->AttatchUpdateCameraParameter(this->calibration2);
		reconstruction1->AttatchMatchedPoint1(&this->refPoints);
		reconstruction1->AttatchMatchedPoint2(&this->scePoints);
		reconstruction1->CalculateNormalizedPoint();
		reconstruction1->ComputeEssentialMatrixRANSAC(&error);
		delete reconstruction1;

		windage::Reconstruction::StereoReconstruction* reconstruction2 = new windage::Reconstruction::StereoReconstruction();
		p2 = (void*)reconstruction2;
		delete reconstruction2;

		sprintf_s(memoryAddress1, "%08X", p1);
		sprintf_s(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		(*message) = std::string(memoryAddress1) + std::string(",") + std::string(memoryAddress2);
		if(compair == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool TestAlgorithm(std::string* message)
	{
		bool test = true;
		char tempMessage[100];
/*
		cvNamedWindow("matching");
		for(int i=0; i<this->scePoints.size(); i++)
		{
			CvPoint pt1 = cvPoint(refPoints[i].GetPoint().x, refPoints[i].GetPoint().y);
			CvPoint pt2 = cvPoint(imageSize.width+scePoints[i].GetPoint().x, scePoints[i].GetPoint().y);

			cvLine(resultImage, pt1, pt2, CV_RGB(255, 0, 0));
		}
		cvShowImage("matching", resultImage);
//*/

		windage::Reconstruction::StereoReconstruction stereoReconstruction;
		stereoReconstruction.AttatchBaseCameraParameter(this->calibration1);
		stereoReconstruction.AttatchUpdateCameraParameter(this->calibration2);
		stereoReconstruction.AttatchMatchedPoint1(&this->refPoints);
		stereoReconstruction.AttatchMatchedPoint2(&this->scePoints);

		double error = 0.0;
		stereoReconstruction.CalculateNormalizedPoint();
		stereoReconstruction.ComputeEssentialMatrixRANSAC(&error);

		int count = stereoReconstruction.GetInlierCount();
		int n = (int)refPoints.size();
		sprintf_s(tempMessage, "%d / %d - error : %.2lf", count, n, error);
		(*message) = std::string(tempMessage);
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		if(inputImage1) cvReleaseImage(&inputImage1);
		inputImage1 = NULL;
		if(inputImage2) cvReleaseImage(&inputImage2);
		inputImage2 = NULL;
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;
		
		cvDestroyWindow("Object Tracking Frameworks");

		return true;
	}
};

