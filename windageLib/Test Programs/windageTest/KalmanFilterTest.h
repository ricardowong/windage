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
#include "Algorithms/KalmanFilter.h"

class KalmanFilterTest : public windageTest
{
private:

public:
	KalmanFilterTest() : windageTest("KalmanFilter Test", "KalmanFilter")
	{
		testImage = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
		this->Do();
	}
	~KalmanFilterTest()
	{
		if(testImage) cvReleaseImage(&testImage);
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters
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
		windage::Algorithms::KalmanFilter* filter1 = new windage::Algorithms::KalmanFilter();
		p1 = (void*)filter1;
		filter1->Predict();
		filter1->Correct(windage::Vector3(0, 0, 0));
		filter1->Predict();
		filter1->Correct(windage::Vector3(0, 0, 0));
		filter1->Predict();
		filter1->Correct(windage::Vector3(0, 0, 0));
		delete filter1;

		windage::Algorithms::KalmanFilter* filter2 = new windage::Algorithms::KalmanFilter();
		p2 = (void*)filter2;
		delete filter2;

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

		CvRNG rng = cvRNG(cvGetTickCount());
		const int X = 500/2;
		const int Y = 500/2;
		const int Z = 500/5;
		const int DELTA = 50;

		cvNamedWindow("Kalman filter");
		windage::Algorithms::KalmanFilter filter;

		for(int i=0; i<50; i++)
		{
			windage::Vector3 prediction = filter.Predict();

			int x = X + (int)cvRandInt(&rng) % DELTA - DELTA/2;
			int y = Y + (int)cvRandInt(&rng) % DELTA - DELTA/2;
			int z = Z + (int)cvRandInt(&rng) % DELTA - DELTA/2;

			filter.Correct(windage::Vector3(x, y, z));

			cvZero(testImage);

			cvCircle(testImage, cvPoint(x, y), z, CV_RGB(0, 255, 0));
			cvCircle(testImage, cvPoint((int)prediction.x, (int)prediction.y), (int)prediction.z, CV_RGB(255, 0, 0));

			cvShowImage("Kalman filter", testImage);
			cvWaitKey(1);
		}
		
		sprintf_s(tempMessage, "");
		(*message) = std::string(tempMessage);
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		if(testImage) cvReleaseImage(&testImage);
				 
		cvDestroyWindow("Kalman filter");

		return true;
	}
};

