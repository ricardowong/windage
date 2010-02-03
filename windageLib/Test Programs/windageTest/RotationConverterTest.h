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

#include "windageTest.h"
#include "Coordinator/RotationConverter.h"

class RotationConverterTest : public windageTest
{
private:
public:
	RotationConverterTest() : windageTest("RotationConverter Test", "RotationConverter")
	{
		this->Do();
	}
	~RotationConverterTest()
	{
	}

	bool Initialize(std::string* message)
	{
		// prepair the test data and setup the parameters

		// not need

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

		// not need

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

		double dx = 15.0 * CV_PI/180.0;
		double dy = 172.0* CV_PI/180.0;
		double dz = (0.0 + 112.0) * CV_PI/180.0;
		
		windage::Vector3 euler			= windage::Vector3(dx, dy, dz);
		windage::Vector4 quaternion		= windage::Coordinator::RotationConverter::EulerToQuaternion(euler);
		windage::Vector3 confirmEuler	= windage::Coordinator::RotationConverter::QuaternionToEuler(quaternion);

		double error1 = euler.getDistance(confirmEuler);
		
		windage::Matrix3 dcm			= windage::Coordinator::RotationConverter::QuaternionToDcm(quaternion);
        windage::Vector4 confirmQuat	= windage::Coordinator::RotationConverter::DcmToQuaternion(dcm);

		double error2 = quaternion.getDistance(confirmQuat);

		dcm								= windage::Coordinator::RotationConverter::EulerToDcm(euler);
        confirmEuler					= windage::Coordinator::RotationConverter::DcmToEuler(dcm);

		double error3 = euler.getDistance(confirmEuler);

		if(error1 > EPS || error2 > EPS || error3 > EPS)
			test = false;
		
		char tempMessage[300];
		sprintf_s(tempMessage, "%lf, %lf, %f", error1, error2, error3);
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