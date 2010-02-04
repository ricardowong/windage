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

#include "windageTest.h"

class windageTestSample : public windageTest
{
private:
public:
	windageTestSample() : windageTest("windage Test Sample", "windageTestSampleClass")
	{
		this->Do();
	}
	~windageTestSample()
	{
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters

		//testImage = cvLoadImage(TEST_IMAGE_FILENAME.c_str());
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

		IplImage* i1 = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_64F, 4);
		p1 = (void*)i1;
		cvReleaseImage(&i1);

		IplImage* i2 = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_64F, 4);
		p2 = (void*)i2;
		cvReleaseImage(&i1);
		
		sprintf_s(memoryAddress1, "%08X", p1);
		sprintf_s(memoryAddress2, "%08X", p2);

		(*message) = std::string(memoryAddress1) + std::string(",") + std::string(memoryAddress2);
		int compair = strcmp(memoryAddress1, memoryAddress2);
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
		// checek the algorithm
		CvRNG rng = cvRNG(cvGetTickCount());
		unsigned int random = cvRandInt(&rng);
		random = random%10+1;

		char temp[100];
		sprintf_s(temp, "randomly fail : %u > 7", random);
		(*message) = std::string(temp);

		if(random > 7)
			return false;
		else
			return true;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);

		return true;
	}
};