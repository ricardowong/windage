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

#include <cv.h>
#include <highgui.h>

#include "windageTest.h"
#include "Algorithms/WSURFdetector.h"
#include "Algorithms/FLANNtree.h"
#include "Algorithms/RANSACestimator.h"
#include "Algorithms/OutlierChecker.h"
#include "Algorithms/InverseCompositional.h"
#include "Utilities/Utils.h"

class InverseCompositionalTest : public windageTest
{
private:
	CvSize imageSize;
	IplImage* grayImage1;
	IplImage* grayImage2;
	windage::Matrix3 homography;

	windage::Algorithms::WSURFdetector* surfDetectorRef;
	windage::Algorithms::WSURFdetector* surfDetectorSce;
	windage::Algorithms::FLANNtree* searchTree;
	windage::Algorithms::RANSACestimator* estimator;

	std::vector<windage::FeaturePoint> referencePoints;
	std::vector<windage::FeaturePoint> scenePoints;

public:
	InverseCompositionalTest() : windageTest("InverseCompositional Test", "InverseCompositional")
	{
		grayImage1 = NULL;
		grayImage2 = NULL;

		surfDetectorRef = NULL;
		surfDetectorSce = NULL;
		searchTree = NULL;
		estimator = NULL;

		homography.m[0][0] = 1.0; homography.m[0][1] = 0.0; homography.m[0][2] = 0.0;
		homography.m[1][0] = 0.0; homography.m[1][1] = 1.0; homography.m[1][2] = 0.0;
		homography.m[2][0] = 0.0; homography.m[2][1] = 0.0; homography.m[2][2] = 1.0;

		this->Do();
	}
	~InverseCompositionalTest()
	{
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;

		if(surfDetectorRef) delete surfDetectorRef;
		surfDetectorRef = NULL;
		if(surfDetectorSce) delete surfDetectorSce;
		surfDetectorSce = NULL;
		if(searchTree) delete searchTree;
		searchTree = NULL;
		if(estimator) delete estimator;
		estimator = NULL;
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters
		// load reference image
		testImage = cvLoadImage(REFERENCE_IMAGE_FILENAME.c_str());
		imageSize.width = testImage->width;
		imageSize.height = testImage->height;

		grayImage1 = cvCreateImage(cvGetSize(testImage), IPL_DEPTH_8U, 1);
		cvCvtColor(testImage, grayImage1, CV_BGR2GRAY);
		cvReleaseImage(&testImage);

		surfDetectorRef = new windage::Algorithms::WSURFdetector();
		surfDetectorRef->DoExtractKeypointsDescriptor(grayImage1);

		// load scene image
		testImage = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		grayImage2 = cvCreateImage(cvGetSize(testImage), IPL_DEPTH_8U, 1);
		cvCvtColor(testImage, grayImage2, CV_BGR2GRAY);
		resultImage = cvCreateImage(cvSize(testImage->width, testImage->height), IPL_DEPTH_8U, 3);
		cvCopyImage(testImage, resultImage);

		surfDetectorSce = new windage::Algorithms::WSURFdetector();
		surfDetectorSce->DoExtractKeypointsDescriptor(grayImage2);
		
		cvReleaseImage(&testImage);
		testImage = NULL;

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

		windage::Algorithms::InverseCompositional* tracker1 = new windage::Algorithms::InverseCompositional(imageSize.width, imageSize.height);
		p1 = (void*)tracker1;
		tracker1->AttatchTemplateImage(grayImage1);
		tracker1->AttatchTemplateImage(grayImage1);
		tracker1->SetSamplingStep(10);
		tracker1->SetInitialHomography(h1);
		tracker1->Initialize();
		tracker1->Initialize();
		tracker1->UpdateHomography(grayImage2);
		tracker1->UpdateHomography(grayImage2);
		tracker1->UpdateHomography(grayImage2);
		tracker1->UpdateHomography(grayImage2);
		delete tracker1;

		windage::Algorithms::InverseCompositional* tracker2 = new windage::Algorithms::InverseCompositional(imageSize.width, imageSize.height);
		p2 = (void*)tracker2;
		tracker2->AttatchTemplateImage(grayImage1);
		tracker2->SetSamplingStep(10);
		tracker2->SetInitialHomography(h2);
		tracker2->Initialize();
		tracker2->UpdateHomography(grayImage2);
		delete tracker2;

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

		const int iterationCount = 50;

		cvNamedWindow("InverseCompositional tracker");

		windage::Matrix3 h = homography;
		std::vector<windage::Matrix3> homographyList;
		for(int i=0; i<2; i++)
			h.m[i][2] += 10.0;

		windage::Algorithms::InverseCompositional tracker(imageSize.width, imageSize.height);
		tracker.AttatchTemplateImage(grayImage1);
		tracker.SetParameterAmplification(5.0);
		tracker.SetSamplingStep(10);
		tracker.SetInitialHomography(h);
		tracker.Initialize();

		double error = 0.0;
		double delta = 0.0;
		for(int i=0; i<iterationCount; i++)
		{
			error = tracker.UpdateHomography(grayImage2, &delta);
			h = tracker.GetHomography();
			homographyList.push_back(h);
		}
		
		int count = homographyList.size();
		for(int i=0; i<count; i++)
			tracker.DrawResult(resultImage, homographyList[i], CV_RGB(((count-i)/(double)count) * 255.0, (i/(double)count) * 255.0, 0.0), 1, imageSize.width, imageSize.height);
		cvShowImage("InverseCompositional tracker", resultImage);
		cvWaitKey(1000);

		sprintf_s(tempMessage, "SSD error : %lf, delta : %lf", error, delta);
		(*message) = std::string(tempMessage);

		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);
		if(grayImage1) cvReleaseImage(&grayImage1);
		grayImage1 = NULL;
		if(grayImage2) cvReleaseImage(&grayImage2);
		grayImage2 = NULL;

		cvDestroyWindow("InverseCompositional tracker");

		return true;
	}
};

