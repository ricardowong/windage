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
#include "Structures/Calibration.h"

class CalibrationTest : public windageTest
{
private:
public:
	CalibrationTest() : windageTest("Calibration Test", "Calibration")
	{
		this->Do();
	}
	~CalibrationTest()
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
		int compair = 0;

		// calibration
		windage::Calibration* calibration1 = new windage::Calibration();
		p1 = (void*)calibration1;
		delete calibration1;

		windage::Calibration* calibration2 = new windage::Calibration();
		p2 = (void*)calibration2;
		delete calibration2;

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
		double EPS = 1.0e-5;
		char tempMessage[100];

		// checek the algorithm
		CvRNG rng = cvRNG(cvGetTickCount());
		windage::Vector4 intrinsic;
		windage::Vector4 distortion;
		windage::Matrix4 extrinsic;
		for(int i=0; i<4; i++)
		{
			intrinsic.v[i] = cvRandReal(&rng);
			distortion.v[i] = cvRandReal(&rng);
			for(int x=0; x<4; x++)
				extrinsic.m[i][x] = cvRandReal(&rng);
		}
		
		windage::Calibration calibration1;
		calibration1.Initialize(intrinsic.x, intrinsic.y, intrinsic.z, intrinsic.w,
								distortion.x, distortion.y, distortion.z, distortion.w);
		calibration1.SetExtrinsicMatrix(extrinsic.m1);

		windage::Calibration calibration2;
		calibration2 = calibration1;

		double error = 0.0;
		for(int i=0; i<4; i++)
		{
			error += abs(calibration2.GetParameters()[i] - intrinsic.v[i]);
			error += abs(calibration2.GetParameters()[4+i] - distortion.v[i]);
			for(int x=0; x<4; x++)
				error += abs(CV_MAT_ELEM((*calibration1.GetExtrinsicMatrix()), double, i, x) - CV_MAT_ELEM((*calibration2.GetExtrinsicMatrix()), double, i, x));
		}
	
		sprintf_s(tempMessage, "error = %.2lf", error);
		(*message) = std::string(tempMessage);
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);

		return true;
	}
};