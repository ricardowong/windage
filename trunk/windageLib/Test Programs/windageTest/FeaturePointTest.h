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
#include "Structures/FeaturePoint.h"
#include "Structures/WSURFpoint.h"
#include "Structures/SURFpoint.h"


class FeaturePointTest : public windageTest
{
private:
public:
	FeaturePointTest() : windageTest("FeaturePoint Test", "FeaturePoint")
	{
		this->Do();
	}
	~FeaturePointTest()
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

		// Feature Point
		windage::FeaturePoint* featurePoint1 = new windage::FeaturePoint();
		p1 = (void*)featurePoint1;
		delete featurePoint1;

		windage::FeaturePoint* featurePoint2 = new windage::FeaturePoint();
		p2 = (void*)featurePoint2;
		delete featurePoint2;

		sprintf(memoryAddress1, "%08X", p1);
		sprintf(memoryAddress2, "%08X", p2);
		compair += strcmp(memoryAddress1, memoryAddress2);

		// Feature Point
		windage::WSURFpoint* wSURFPoint1 = new windage::WSURFpoint();
		p1 = (void*)wSURFPoint1;
		delete wSURFPoint1;

		windage::WSURFpoint* wSURFPoint2 = new windage::WSURFpoint();
		p2 = (void*)wSURFPoint2;
		delete wSURFPoint2;

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
		int dimension = 36;

		// checek the algorithm
		CvRNG rng = cvRNG(cvGetTickCount());
		
		double px = (double)(cvRandInt(&rng) % 200 - 100);
		double py = (double)(cvRandInt(&rng) % 200 - 100);
		double pz = (double)(cvRandInt(&rng) % 200 - 100);

		int id = (int)(cvRandInt(&rng) % 200 - 100);
		int size = (int)(cvRandInt(&rng) % 200 - 100);
		double dir = (double)(cvRandInt(&rng) % 200 - 100);

		double* d = new double[dimension];
		for(int i=0; i<dimension; i++)
			d[i] = (double)(cvRandInt(&rng) % 200 - 100);

		// FeaturePoint
		windage::FeaturePoint featurePoint;
		featurePoint.SetPoint(windage::Vector3(px, py, pz));
		featurePoint.SetObjectID(id);
		featurePoint.SetSize(size);
		featurePoint.SetDir(dir);
		for(int i=0; i<featurePoint.DESCRIPTOR_DIMENSION; i++)
			featurePoint.descriptor[i] = d[i];
		windage::FeaturePoint tempFeauterPoint = featurePoint;

		windage::Vector3 deltaPoint = tempFeauterPoint.GetPoint() - featurePoint.GetPoint();
		int deltaID = tempFeauterPoint.GetObjectID() - featurePoint.GetObjectID();
		double deltaDescriptor = tempFeauterPoint.GetDistance(featurePoint);
		int deltaSize = tempFeauterPoint.GetSize() - featurePoint.GetSize();
		double deltaDir = tempFeauterPoint.GetDir() - featurePoint.GetDir();

		if(deltaPoint.getLength() > EPS)
			test = false;
		if(deltaID != 0)
			test = false;
		if(deltaDescriptor > EPS)
			test = false;
		if(deltaSize != 0)
			test = false;
		if(deltaDir > EPS)
			test = false;

		// wSURFpoint
		windage::WSURFpoint wSURFPoint;
		wSURFPoint.SetPoint(windage::Vector3(px, py, pz));
		wSURFPoint.SetObjectID(id);
		wSURFPoint.SetSize(size);
		wSURFPoint.SetDir(dir);
		for(int i=0; i<wSURFPoint.DESCRIPTOR_DIMENSION; i++)
			wSURFPoint.descriptor[i] = d[i];
		windage::WSURFpoint tempWSURFPoint = wSURFPoint;

		deltaPoint = tempWSURFPoint.GetPoint() - wSURFPoint.GetPoint();
		deltaID = tempWSURFPoint.GetObjectID() - wSURFPoint.GetObjectID();
		deltaDescriptor = tempWSURFPoint.GetDistance(wSURFPoint);
		deltaSize = tempWSURFPoint.GetSize() - wSURFPoint.GetSize();
		deltaDir = tempWSURFPoint.GetDir() - wSURFPoint.GetDir();

		if(deltaPoint.getLength() > EPS)
			test = false;
		if(deltaID != 0)
			test = false;
		if(deltaDescriptor > EPS)
			test = false;
		if(deltaSize != 0)
			test = false;
		if(deltaDir > EPS)
			test = false;

		// SURFpoint
		windage::SURFpoint surfPoint;
		surfPoint.SetPoint(windage::Vector3(px, py, pz));
		surfPoint.SetObjectID(id);
		surfPoint.SetSize(size);
		surfPoint.SetDir(dir);
		for(int i=0; i<surfPoint.DESCRIPTOR_DIMENSION; i++)
			surfPoint.descriptor[i] = d[i];
		windage::SURFpoint tempSurfPoint = surfPoint;

		deltaPoint = tempSurfPoint.GetPoint() - surfPoint.GetPoint();
		deltaID = tempSurfPoint.GetObjectID() - surfPoint.GetObjectID();
		deltaDescriptor = tempSurfPoint.GetDistance(surfPoint);
		deltaSize = tempSurfPoint.GetSize() - surfPoint.GetSize();
		deltaDir = tempSurfPoint.GetDir() - surfPoint.GetDir();

		if(deltaPoint.getLength() > EPS)
			test = false;
		if(deltaID != 0)
			test = false;
		if(deltaDescriptor > EPS)
			test = false;
		if(deltaSize != 0)
			test = false;
		if(deltaDir > EPS)
			test = false;

		delete[] d;
	
		return test;
	}

	bool Terminate(std::string* message)
	{
		// remove data and reset the parameters
		//if(testImage) cvReleaseImage(&testImage);

		return true;
	}
};