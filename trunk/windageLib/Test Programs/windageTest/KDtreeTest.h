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
#include "Algorithms/KDtree.h"
#include "Utilities/Utils.h"

class KDtreeTest : public windageTest
{
private:
	IplImage* grayImage;
	windage::Algorithms::WSURFdetector* surfDetectorRef;
	windage::Algorithms::WSURFdetector* surfDetectorSce;

public:
	KDtreeTest() : windageTest("KDtree Test", "KDtree")
	{
		grayImage = NULL;
		surfDetectorRef = NULL;
		surfDetectorSce = NULL;
		this->Do();
	}
	~KDtreeTest()
	{
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		if(surfDetectorRef) delete surfDetectorRef;
		surfDetectorRef = NULL;
		if(surfDetectorSce) delete surfDetectorSce;
		surfDetectorSce = NULL;
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

		// Feature Point
		windage::Algorithms::KDtree* tree1 = new windage::Algorithms::KDtree();
		p1 = (void*)tree1;
		tree1->Training(surfDetectorRef->GetKeypoints());
		tree1->Training(surfDetectorRef->GetKeypoints());
		tree1->Training(surfDetectorRef->GetKeypoints());
		std::vector<windage::FeaturePoint>* scenePoints = surfDetectorSce->GetKeypoints();
		for(int i=0; i<scenePoints->size(); i++)
		{
			int index = tree1->Matching((*scenePoints)[i]);
		}
		tree1->Training(surfDetectorRef->GetKeypoints());
		for(int i=0; i<scenePoints->size(); i++)
		{
			int index = tree1->Matching((*scenePoints)[i]);
		}
		delete tree1;

		windage::Algorithms::KDtree* tree2 = new windage::Algorithms::KDtree();
		p2 = (void*)tree2;
		tree2->Training(surfDetectorSce->GetKeypoints());
		delete tree2;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
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

		int width = resultImage->width / 2;
		
		windage::Algorithms::KDtree kdtree;
		kdtree.Training(surfDetectorRef->GetKeypoints());
		std::vector<windage::FeaturePoint>* scenePoints = surfDetectorSce->GetKeypoints();
		for(int i=0; i<scenePoints->size(); i++)
		{
			double distance = 1.0e10;
			int index = kdtree.Matching((*scenePoints)[i], &distance);
			if(index >= 0)
			{
				windage::Vector3 refPT = (*surfDetectorRef->GetKeypoints())[index].GetPoint();
				CvPoint pointRef = cvPoint(refPT.x, refPT.y);

				windage::Vector3 scePT = (*surfDetectorSce->GetKeypoints())[i].GetPoint();
				CvPoint pointSce = cvPoint(scePT.x + width, scePT.y);

				cvLine(resultImage, pointRef, pointSce, CV_RGB(0, 255, 0));
			}
		}

		cvNamedWindow("KD tree search");
		cvShowImage("KD tree search", resultImage);
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
		 
		cvDestroyWindow("KD tree search");

		return true;
	}
};

