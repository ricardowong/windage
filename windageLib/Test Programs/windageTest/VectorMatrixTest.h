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
#include "Structures/Vector.h"
#include "Structures/Matrix.h"

class VectorMatrixTest : public windageTest
{
private:
public:
	VectorMatrixTest() : windageTest("Vector&Matrix Test", "Vector&Matrix")
	{
		this->Do();
	}
	~VectorMatrixTest()
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

		// Vector2
		windage::Vector2* v21 = new windage::Vector2();
		p1 = (void*)v21;
		delete v21;

		windage::Vector2* v22 = new windage::Vector2();
		p2 = (void*)v22;
		delete v22;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Vector3
		windage::Vector3* v31 = new windage::Vector3();
		p1 = (void*)v31;
		delete v31;

		windage::Vector3* v32 = new windage::Vector3();
		p2 = (void*)v32;
		delete v32;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Vector4
		windage::Vector4* v41 = new windage::Vector4();
		p1 = (void*)v41;
		delete v41;

		windage::Vector4* v42 = new windage::Vector4();
		p2 = (void*)v42;
		delete v42;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Matrix2
		windage::Matrix2* m21 = new windage::Matrix2();
		p1 = (void*)m21;
		delete m21;

		windage::Matrix2* m22 = new windage::Matrix2();
		p2 = (void*)m22;
		delete m22;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Matrix3
		windage::Matrix3* m31 = new windage::Matrix3();
		p1 = (void*)m31;
		delete m31;

		windage::Matrix3* m32 = new windage::Matrix3();
		p2 = (void*)m32;
		delete m32;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Matrix4
		windage::Matrix4* m41 = new windage::Matrix4();
		p1 = (void*)m41;
		delete m41;

		windage::Matrix4* m42 = new windage::Matrix4();
		p2 = (void*)m42;
		delete m42;

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
		double EPS = 1.0e-5;

		// checek the algorithm
		CvRNG rng = cvRNG(cvGetTickCount());
		double r11 = (double)1;//cvRandInt(&rng) % 200 - 100;
		double r12 = (double)((int)cvRandInt(&rng) % 200 - 100);
		double r13 = 0;//cvRandInt(&rng) % 200 - 100;
		double r14 = 0;

		double r21 = r12;
		double r22 = (double)((int)cvRandInt(&rng) % 200 - 100);
		double r23 = (double)((int)cvRandInt(&rng) % 200 - 100);
		double r24 = 0;//cvRandInt(&rng) % 200 - 100;

		double r31 = r13;
		double r32 = r23;
		double r33 = (double)((int)cvRandInt(&rng) % 200 - 100);
		double r34 = (double)((int)cvRandInt(&rng) % 200 - 100);

		double r41 = r14;
		double r42 = r24;
		double r43 = r34;
		double r44 = (double)((int)cvRandInt(&rng) % 200 - 100);

		int r1 = cvRandInt(&rng) % 200 - 100;
		int r2 = cvRandInt(&rng) % 200 - 100;
		int r3 = cvRandInt(&rng) % 200 - 100;
		int r4 = cvRandInt(&rng) % 200 - 100;

		windage::Vector2 v21 = windage::Vector2(r1, r2);
		windage::Matrix2 m21 = windage::Matrix2(r11, r12, r21, r22);
		windage::Vector2 v22 = m21 * v21;
		windage::Vector2 v2r = m21.Inverse() * v22;
		v2r = v2r - v21;

		double error1 = v2r.getLength();
		if(error1 > EPS)
			test = false;

		windage::Vector3 v31 = windage::Vector3(r1, r2, r3);
		windage::Matrix3 m31 = windage::Matrix3(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		windage::Vector3 v32 = m31 * v31;
		windage::Vector3 v3r = m31.Inverse() * v32;
		v3r = v3r - v31;

		double error2 = v3r.getLength();
		if(error2 > EPS)
			test = false;

		windage::Vector4 v41 = windage::Vector4(r1, r2, r3, r4);
		windage::Matrix4 m41 = windage::Matrix4(r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, r41, r42, r43, r44);
		windage::Vector4 v42 = m41 * v41;
		windage::Vector4 v4r = m41.Inverse() * v42;
		v4r = v4r - v41;

		double error3 = v4r.getLength();
		if(error3 > EPS)
			test = false;

		char tempMessage[300];
		sprintf(tempMessage, "%lf, %lf, %f", error1, error2, error3);
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