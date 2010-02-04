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
#include "Algorithms/WSURFdetector.h"
#include "Algorithms/FLANNtree.h"
#include "Algorithms/RANSACestimator.h"
#include "Algorithms/OutlierChecker.h"
#include "Algorithms/LMmethod.h"
#include "Utilities/Utils.h"

class LMmethodTest : public windageTest
{
private:
	IplImage* grayImage;
	windage::Matrix3 homography;

	windage::Algorithms::WSURFdetector* surfDetectorRef;
	windage::Algorithms::WSURFdetector* surfDetectorSce;
	windage::Algorithms::FLANNtree* searchTree;
	windage::Algorithms::RANSACestimator* estimator;
	windage::Algorithms::OutlierChecker* checker;

	std::vector<windage::FeaturePoint> referencePoints;
	std::vector<windage::FeaturePoint> scenePoints;

public:
	LMmethodTest() : windageTest("LMmethod Refiner Test", "LMmethod")
	{
		grayImage = NULL;
		surfDetectorRef = NULL;
		surfDetectorSce = NULL;
		searchTree = NULL;
		estimator = NULL;
		checker = NULL;

		this->Do();
	}
	~LMmethodTest()
	{
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		if(surfDetectorRef) delete surfDetectorRef;
		surfDetectorRef = NULL;
		if(surfDetectorSce) delete surfDetectorSce;
		surfDetectorSce = NULL;
		if(searchTree) delete searchTree;
		searchTree = NULL;
		if(estimator) delete estimator;
		estimator = NULL;
		if(checker) delete checker;
		checker = NULL;

		referencePoints.clear();
		scenePoints.clear();
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters

		// load reference image
		testImage = cvLoadImage(REFERENCE_IMAGE_FILENAME.c_str());
		grayImage = cvCreateImage(cvGetSize(testImage), IPL_DEPTH_8U, 1);
		cvCvtColor(testImage, grayImage, CV_BGR2GRAY);

		resultImage = cvCreateImage(cvSize(testImage->width * 2, testImage->height), IPL_DEPTH_8U, 3);
		cvSetImageROI(resultImage, cvRect(0, 0, testImage->width, testImage->height));
		cvCopyImage(testImage, resultImage);

		surfDetectorRef = new windage::Algorithms::WSURFdetector();
		surfDetectorRef->DoExtractKeypointsDescriptor(grayImage);
		cvReleaseImage(&testImage);

		// load scene image
		testImage = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		cvCvtColor(testImage, grayImage, CV_BGR2GRAY);

		cvSetImageROI(resultImage, cvRect(testImage->width, 0, testImage->width, testImage->height));
		cvCopyImage(testImage, resultImage);

		surfDetectorSce = new windage::Algorithms::WSURFdetector();
		surfDetectorSce->DoExtractKeypointsDescriptor(grayImage);
		cvReleaseImage(&testImage);

		cvResetImageROI(resultImage);

		// matching : find corresponding points
		searchTree = new windage::Algorithms::FLANNtree();
		searchTree->Training(surfDetectorRef->GetKeypoints());
		std::vector<windage::FeaturePoint>* pScenePoints = surfDetectorSce->GetKeypoints();
		for(unsigned int i=0; i<pScenePoints->size(); i++)
		{
			double distance = 1.0e10;
			int index = searchTree->Matching((*pScenePoints)[i], &distance);
			if(index >= 0)
			{
				windage::FeaturePoint ref;
				windage::FeaturePoint sce;

				ref = (*surfDetectorRef->GetKeypoints())[index];
				ref.SetDistance(distance);
				sce = (*pScenePoints)[i];
				sce.SetDistance(distance);

				referencePoints.push_back(ref);
				scenePoints.push_back(sce);
			}
		}

		// Pose estimator
		estimator = new windage::Algorithms::RANSACestimator();
		estimator->AttatchReferencePoint(&this->referencePoints);
		estimator->AttatchScenePoint(&this->scenePoints);
		estimator->Calculate();

		homography = (*estimator->GetHomography());

		checker = new windage::Algorithms::OutlierChecker();
		checker->AttatchEstimator(estimator);
		checker->Calculate();

		for(int i=0; i<(int)this->referencePoints.size(); i++)
		{
			if(this->referencePoints[i].IsOutlier() == true)
			{
				this->referencePoints.erase(this->referencePoints.begin() + i);
				this->scenePoints.erase(this->scenePoints.begin() + i);
				i--;
			}
		}

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

		windage::Matrix3 h1 = homography;
		windage::Matrix3 h2 = homography;

		windage::Algorithms::LMmethod* refiner1 = new windage::Algorithms::LMmethod();
		p1 = (void*)refiner1;
		refiner1->AttatchHomography(&h1);
		refiner1->AttatchReferencePoint(&this->referencePoints);
		refiner1->AttatchScenePoint(&this->scenePoints);
		refiner1->Calculate();
		delete refiner1;

		windage::Algorithms::LMmethod* refiner2 = new windage::Algorithms::LMmethod();
		p2 = (void*)refiner2;
		refiner2->AttatchHomography(&h2);
		refiner2->AttatchReferencePoint(&this->referencePoints);
		refiner2->AttatchScenePoint(&this->scenePoints);
		refiner2->Calculate();
		delete refiner2;

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

		windage::Matrix3 h = homography;

		windage::Algorithms::LMmethod refiner;
		refiner.AttatchHomography(&h);
		refiner.AttatchReferencePoint(&this->referencePoints);
		refiner.AttatchScenePoint(&this->scenePoints);
		refiner.Calculate();

		double errorH = 0.0;
		double errorR = 0.0;
		for(unsigned int i=0; i<this->referencePoints.size(); i++)
		{
			windage::Vector3 world = this->referencePoints[i].GetPoint();
			windage::Vector3 image = this->scenePoints[i].GetPoint();

			windage::Vector3 resultH = homography * world;
			windage::Vector3 resultR = h * world;

			resultH /= resultH.z;
			resultR /= resultR.z;

			errorH += image.getDistance(resultH);
			errorR += image.getDistance(resultR);
		}

		sprintf_s(tempMessage, "original : %lf, refine : %lf", errorH, errorR);
		(*message) = std::string(tempMessage);
		if(errorH < errorR)
			return false;

		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		if(surfDetectorRef) delete surfDetectorRef;
		surfDetectorRef = NULL;
		if(surfDetectorSce) delete surfDetectorSce;
		surfDetectorSce = NULL;
		if(estimator) delete estimator;
		estimator = NULL;

		referencePoints.clear();
		scenePoints.clear();

		return true;
	}
};

