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

#include <iostream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "../Utils/Utils.h"
#include "../Algorithms/homographyESM.h"

const int IMAGE_SEQ_COUNT = 200;
const char* IMAGE_SEQ_FILE_NAME = "seq/im%03d.pgm";

const int GAUSSIAN_BLUR = 5;

const int TEMPLATE_WIDTH = 150;
const int TEMPLATE_HEIGHT = 150;
const float HOMOGRAPHY_DELTA = 0.01f;
const int MAX_ITERATION = 30;

void DrawResult(IplImage* image, windage::Matrix3 homography, CvScalar color = CV_RGB(255, 0, 0), int thickness = 1)
{
	windage::Vector3 pt1(0.0, 0.0, 1.0);
	windage::Vector3 pt2(TEMPLATE_WIDTH, 0.0, 1.0);
	windage::Vector3 pt3(TEMPLATE_WIDTH, TEMPLATE_HEIGHT, 1.0);
	windage::Vector3 pt4(0.0, TEMPLATE_HEIGHT, 1.0);

	windage::Vector3 outPoint1 = homography * pt1;
	windage::Vector3 outPoint2 = homography * pt2;
	windage::Vector3 outPoint3 = homography * pt3;
	windage::Vector3 outPoint4 = homography * pt4;

	outPoint1 /= outPoint1.z;
	outPoint2 /= outPoint2.z;
	outPoint3 /= outPoint3.z;
	outPoint4 /= outPoint4.z;

	cvLine(image, cvPoint((int)outPoint1.x, (int)outPoint1.y), cvPoint((int)outPoint2.x, (int)outPoint2.y), color, thickness);
	cvLine(image, cvPoint((int)outPoint2.x, (int)outPoint2.y), cvPoint((int)outPoint3.x, (int)outPoint3.y), color, thickness);		
	cvLine(image, cvPoint((int)outPoint3.x, (int)outPoint3.y), cvPoint((int)outPoint4.x, (int)outPoint4.y), color, thickness);
	cvLine(image, cvPoint((int)outPoint4.x, (int)outPoint4.y), cvPoint((int)outPoint1.x, (int)outPoint1.y), color, thickness);
}

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("sampling");
	cvNamedWindow("result");

	// initialize
	sprintf_s(message, IMAGE_SEQ_FILE_NAME, 0);
	IplImage* saveImage = cvLoadImage(message, 0);
	if(GAUSSIAN_BLUR > 0)
		cvSmooth(saveImage, saveImage, CV_GAUSSIAN, GAUSSIAN_BLUR, GAUSSIAN_BLUR);

	int width = saveImage->width;
	int height = saveImage->height;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	float q = TEMPLATE_WIDTH * TEMPLATE_HEIGHT;
	
	IplImage* inputImage = NULL;
	IplImage* resultImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage* templateImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* samplingImage = NULL;

	// set template image
	CvRect rect = cvRect(startX, startY, TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	cvSetImageROI(saveImage, rect);
	cvCopyImage(saveImage, templateImage);
	cvShowImage("template", templateImage);
	cvResetImageROI(saveImage);
	cvReleaseImage(&saveImage);

	// initial homography
	windage::Matrix3 homography(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	homography._13 = startX;
	homography._23 = startY;
	windage::Matrix3 e = homography;

	// Template based Tracking using Homography ESM
	windage::HomographyESM* esm = new windage::HomographyESM(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	esm->AttatchTemplateImage(templateImage);
	esm->SetInitialHomography(e);
	esm->Initialize();

	// homography update stack
	std::vector<windage::Matrix3> homographyList;

	cvWaitKey();

	float sumTime = 0.0;
	int sumIter = 0;
	bool processing =true;
	int k = 0;
	while(processing)
	{
		if(k >= IMAGE_SEQ_COUNT)
			processing = false;

		// load image
		if(inputImage) cvReleaseImage(&inputImage);
		sprintf_s(message, IMAGE_SEQ_FILE_NAME, k);
		inputImage = cvLoadImage(message, 0);
		if(GAUSSIAN_BLUR > 0)
			cvSmooth(inputImage, inputImage, CV_GAUSSIAN, GAUSSIAN_BLUR, GAUSSIAN_BLUR);

		cvCvtColor(inputImage, resultImage, CV_GRAY2BGR);

		// processing
		int64 startTime = cvGetTickCount();

		float error = 0.0;
		float delta = 1.0;
		int iter = 0;
		homographyList.clear();
		for(iter=0; iter<MAX_ITERATION; iter++)
		{
			error = esm->UpdateHomography(inputImage, &delta);
			homography = esm->GetHomography();
			samplingImage = esm->GetSamplingImage();

			homographyList.push_back(homography);

//			if(delta < HOMOGRAPHY_DELTA)
//				break;
		}
		int64 endTime = cvGetTickCount();
		k++;
		sumIter+= iter;
		
		// draw result
		int count = homographyList.size();
		int i = count - 1;
		for(i=0; i<count; i++)
 			DrawResult(resultImage, homographyList[i], CV_RGB(((count-i)/(float)count) * 255.0, (i/(float)count) * 255.0, 0), 1);

		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		sprintf_s(message, "%03d >> processing time : %.2lf ms (%02d iter), error : %.2lf", k, processingTime, iter, error);
		std::cout << message << std::endl;
		sumTime += processingTime;

		windage::Utils::DrawTextToImage(resultImage, cvPoint(5, 15), message, 0.6);

		// draw image
		cvShowImage("sampling", samplingImage);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			{
				char tempch = cvWaitKey(0);
				if(tempch == 's' || tempch == 'S')
				{
					cvSaveImage("ESMAlgorithm.jpg", resultImage);
				}
			}
			break;
		case 'i':
		case 'I':
			k++;
			break;
		case 'o':
		case 'O':
			k-=5;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	std::cout << "average iteration : " << sumIter/(float)IMAGE_SEQ_COUNT << std::endl;
	std::cout << "average processing ime : " << sumTime/(float)IMAGE_SEQ_COUNT << " ms" << std::endl;

	cvReleaseImage(&resultImage);
	cvDestroyAllWindows();
}