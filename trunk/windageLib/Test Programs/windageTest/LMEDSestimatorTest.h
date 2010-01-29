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
#include "Algorithms/SURFdetector.h"
#include "Algorithms/FLANNtree.h"
#include "Algorithms/LMEDSestimator.h"
#include "Utilities/Utils.h"

class LMEDSestimatorTest : public windageTest
{
private:
	IplImage* grayImage;
	windage::Algorithms::SURFdetector* surfDetectorRef;
	windage::Algorithms::SURFdetector* surfDetectorSce;
	windage::Algorithms::FLANNtree* searchTree;

	std::vector<windage::FeaturePoint*> referencePoints;
	std::vector<windage::FeaturePoint*> scenePoints;

public:
	LMEDSestimatorTest() : windageTest("LMEDSestimator Test", "LMEDSestimator")
	{
		grayImage = NULL;
		surfDetectorRef = NULL;
		surfDetectorSce = NULL;
		searchTree = NULL;
		this->Do();
	}
	~LMEDSestimatorTest()
	{
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		if(surfDetectorRef) delete surfDetectorRef;
		surfDetectorRef = NULL;
		if(surfDetectorSce) delete surfDetectorSce;
		surfDetectorSce = NULL;
		if(searchTree) delete searchTree;
		searchTree = NULL;

		for(int i=0; i<referencePoints.size(); i++)
			delete referencePoints[i];
		referencePoints.clear();
		for(int i=0; i<scenePoints.size(); i++)
			delete scenePoints[i];
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

		surfDetectorRef = new windage::Algorithms::SURFdetector(2000.0);
		surfDetectorRef->DoExtractKeypointsDescriptor(grayImage);
		cvReleaseImage(&testImage);

		// load scene image
		testImage = cvLoadImage(MATCHING_IMAGE_FILENAME.c_str());
		cvCvtColor(testImage, grayImage, CV_BGR2GRAY);

		cvSetImageROI(resultImage, cvRect(testImage->width, 0, testImage->width, testImage->height));
		cvCopyImage(testImage, resultImage);

		surfDetectorSce = new windage::Algorithms::SURFdetector(2000.0);
		surfDetectorSce->DoExtractKeypointsDescriptor(grayImage);
		cvReleaseImage(&testImage);

		cvResetImageROI(resultImage);

		// matching : find corresponding points
		searchTree = new windage::Algorithms::FLANNtree();
		searchTree->Training(surfDetectorRef->GetKeypoints());
		std::vector<windage::FeaturePoint*>* pScenePoints = surfDetectorSce->GetKeypoints();
		for(int i=0; i<pScenePoints->size(); i++)
		{
			double distance = 1.0e10;
			int index = searchTree->Matching((*(*pScenePoints)[i]), &distance);
			if(index >= 0)
			{
				windage::FeaturePoint* ref = new windage::FeaturePoint();
				windage::FeaturePoint* sce = new windage::FeaturePoint();

				(*ref) = (*(*surfDetectorRef->GetKeypoints())[index]);
				ref->SetDistance(distance);
				(*sce) = (*(*pScenePoints)[i]);
				sce->SetDistance(distance);

				referencePoints.push_back(ref);
				scenePoints.push_back(sce);
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

		windage::Algorithms::LMEDSestimator* estimator1 = new windage::Algorithms::LMEDSestimator();
		p1 = (void*)estimator1;
		estimator1->AttatchReferencePoint(&this->referencePoints);
		estimator1->AttatchScenePoint(&this->scenePoints);
		estimator1->Calculate();
		delete estimator1;

		windage::Algorithms::LMEDSestimator* estimator2 = new windage::Algorithms::LMEDSestimator();
		p2 = (void*)estimator2;
		estimator2->AttatchReferencePoint(&this->referencePoints);
		estimator2->AttatchScenePoint(&this->scenePoints);
		estimator2->Calculate();
		delete estimator2;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

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

		int width = resultImage->width / 2;
		int height = resultImage->height;

		cvNamedWindow("LMEDS estimator search");
		
		windage::Algorithms::LMEDSestimator estimator;
		estimator.AttatchReferencePoint(&this->referencePoints);
		estimator.AttatchScenePoint(&this->scenePoints);
		estimator.Calculate();

		windage::Vector3 drawRefPoints[4];
		windage::Vector3 drawScePoints[4];
		drawRefPoints[0].x = 0.0;	drawRefPoints[0].y = 0.0;		drawRefPoints[0].z = 1.0;
		drawRefPoints[1].x = width; drawRefPoints[1].y = 0.0;		drawRefPoints[1].z = 1.0;
		drawRefPoints[2].x = width; drawRefPoints[2].y = height;	drawRefPoints[2].z = 1.0;
		drawRefPoints[3].x = 0.0;	drawRefPoints[3].y = height;	drawRefPoints[3].z = 1.0;

		for(int i=0; i<4; i++)
		{
			drawScePoints[i] = estimator.ConvertObjectToImage(drawRefPoints[i]);
			drawScePoints[i] /= drawScePoints[i].z;
		}

		for(int i=0; i<4; i++)
		{
			int i2 = i==3?0:i+1;
			cvLine(resultImage, cvPoint(width + drawScePoints[i].x, drawScePoints[i].y),
								cvPoint(width + drawScePoints[i2].x, drawScePoints[i2].y), CV_RGB(0, 255, 0), 3);
		}

		cvShowImage("LMEDS estimator search", resultImage);
		cvWaitKey(1000);

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

		for(int i=0; i<referencePoints.size(); i++)
			delete referencePoints[i];
		referencePoints.clear();
		for(int i=0; i<scenePoints.size(); i++)
			delete scenePoints[i];
		scenePoints.clear();
		
		cvDestroyWindow("LMEDS estimator search");

		return true;
	}
};

