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
#include "Utilities/Utils.h"

class SURFdetectorTest : public windageTest
{
private:
	IplImage* grayImage;

public:
	SURFdetectorTest() : windageTest("SURFdetector Test", "SURFdetector")
	{
		grayImage = NULL;
		this->Do();
	}
	~SURFdetectorTest()
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
		windage::Algorithms::SURFdetector* surfDetector1 = new windage::Algorithms::SURFdetector();
		p1 = (void*)surfDetector1;
		surfDetector1->DoExtractKeypointsDescriptor(grayImage);
		delete surfDetector1;

		windage::Algorithms::SURFdetector* surfDetector2 = new windage::Algorithms::SURFdetector();
		p2 = (void*)surfDetector2;
		surfDetector1->DoExtractKeypointsDescriptor(grayImage);
		delete surfDetector2;

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
		windage::Algorithms::SURFdetector surfDetector;

		cvNamedWindow("SURF detector");
		for(int i=1; i<10; i++)
		{
			double threshold = 500.0 * (double)i;
			cvCopyImage(testImage, resultImage);

			surfDetector.SetThreshold(threshold);
			surfDetector.DoExtractKeypointsDescriptor(grayImage);
			surfDetector.DrawKeypoints(resultImage, CV_RGB(255, 0, 0));

			sprintf(tempMessage, "SURF threshold : %.2lf", threshold);
			windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.7, tempMessage);

			cvShowImage("SURF detector", resultImage);
			cvWaitKey(1);
		}
		
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);
		if(grayImage) cvReleaseImage(&grayImage);
		grayImage = NULL;
		cvDestroyWindow("SURF detector");

		return true;
	}
};

