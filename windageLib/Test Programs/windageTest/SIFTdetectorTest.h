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
#include "Algorithms/SIFTdetector.h"
#include "Utilities/Utils.h"

class SIFTdetectorTest : public windageTest
{
private:
	IplImage* grayImage;

public:
	SIFTdetectorTest() : windageTest("SIFTdetector Test", "SIFTdetector")
	{
		grayImage = NULL;
		this->Do();
	}
	~SIFTdetectorTest()
	{
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters
		testImage = cvLoadImage(TEST_IMAGE_FILENAME.c_str());
		grayImage = cvCreateImage(cvGetSize(testImage), IPL_DEPTH_8U, 1);
		resultImage = cvCreateImage(cvGetSize(testImage), IPL_DEPTH_8U, 3);
		cvCvtColor(testImage, grayImage, CV_BGR2GRAY);

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
		windage::Algorithms::SIFTdetector* siftDetector1 = new windage::Algorithms::SIFTdetector();
		p1 = (void*)siftDetector1;
		siftDetector1->DoExtractKeypointsDescriptor(grayImage);
		delete siftDetector1;

		windage::Algorithms::SIFTdetector* siftDetector2 = new windage::Algorithms::SIFTdetector();
		p2 = (void*)siftDetector2;
		siftDetector2->DoExtractKeypointsDescriptor(grayImage);
		delete siftDetector2;

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
		windage::Algorithms::SIFTdetector SIFTDetector;

		cvNamedWindow("SIFT detector");
		cvCopyImage(testImage, resultImage);

		SIFTDetector.DoExtractKeypointsDescriptor(grayImage);
		SIFTDetector.DrawKeypoints(resultImage, CV_RGB(255, 0, 0));

		sprintf_s(tempMessage, "SIFT detector");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.7, tempMessage);

		cvShowImage("SIFT detector", resultImage);
		cvWaitKey(1000);
		
		sprintf_s(tempMessage, "");(*message) = std::string(tempMessage);
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		cvDestroyWindow("SIFT detector");

		return true;
	}
};

