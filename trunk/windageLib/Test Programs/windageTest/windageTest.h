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

#ifndef _WINDAGE_TEST_H_
#define _WINDAGE_TEST_H_

#include <string>
#include <iostream>

#include <cv.h>
#include <highgui.h>

class windageTest
{
public:
	std::string TEST_IMAGE_FILENAME;
	std::string REFERENCE_IMAGE_FILENAME;
	std::string MATCHING_IMAGE_FILENAME;

protected:
	std::string testName;
	std::string testClass;

	IplImage* testImage;
	IplImage* resultImage;

public:
	windageTest(const std::string testName, const std::string testClass)
	{
		TEST_IMAGE_FILENAME = "Test/testReference.png";
		REFERENCE_IMAGE_FILENAME = "Test/testImage1.png";
		MATCHING_IMAGE_FILENAME = "Test/testImage2.png";

		this->testName = testName;
		this->testClass = testClass;

		testImage = NULL;
		resultImage = NULL;
	}
	virtual ~windageTest()
	{
		if(testImage) cvReleaseImage(&testImage);
		if(resultImage) cvReleaseImage(&resultImage);
	}

	void Do();
	virtual bool Initialize(std::string* message) = 0;
	virtual bool TestMemoryRelease(std::string* message) = 0;
	virtual bool TestAlgorithm(std::string* message) = 0;
	virtual bool Terminate(std::string* message) = 0;
};

#endif