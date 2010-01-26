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

#include "../Algorithms/homographyESM.h"

const int IMAGE_SEQ_COUNT = 200;
const char* IMAGE_SEQ_FILE_NAME = "seq/im%03d.pgm";

const double PROCESSING_TIME = 33.0 * 3;//ms

const int TEMPLATE_WIDTH = 200;
const int TEMPLATE_HEIGHT = 200;

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("sampling");
	cvNamedWindow("result");

	// initialize
	sprintf(message, IMAGE_SEQ_FILE_NAME, 0);
	IplImage* saveImage = cvLoadImage(message, 0);
	cvSmooth(saveImage, saveImage, CV_GAUSSIAN, 3, 3);

	int width = saveImage->width;
	int height = saveImage->height;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	double q = TEMPLATE_WIDTH * TEMPLATE_HEIGHT;
	
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

	bool processing =true;
	int k = 0;
	while(processing)
	{
		if(k >= IMAGE_SEQ_COUNT)
			processing = false;

		// load image
		if(inputImage) cvReleaseImage(&inputImage);
		sprintf(message, IMAGE_SEQ_FILE_NAME, k);
		inputImage = cvLoadImage(message, 0);
		cvSmooth(inputImage, inputImage, CV_GAUSSIAN, 3, 3);

		cvCvtColor(inputImage, resultImage, CV_GRAY2BGR);

		// processing
		int64 startTime = cvGetTickCount();
		int64 endTime;

		double error = 0.0;
		bool roop = true;
//		while(roop)
		{
			error = esm->UpdateHomography(inputImage);
			homography = esm->GetHomography();
			samplingImage = esm->GetSamplingImage();

			endTime = cvGetTickCount();
		}
//		k++;

		// draw result
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

		cvLine(resultImage, cvPoint(outPoint1.x, outPoint1.y), cvPoint(outPoint2.x, outPoint2.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint2.x, outPoint2.y), cvPoint(outPoint3.x, outPoint3.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint3.x, outPoint3.y), cvPoint(outPoint4.x, outPoint4.y), CV_RGB(255, 0, 0));
		cvLine(resultImage, cvPoint(outPoint4.x, outPoint4.y), cvPoint(outPoint1.x, outPoint1.y), CV_RGB(255, 0, 0));

		// draw image
		cvShowImage("sampling", samplingImage);
		cvShowImage("result", resultImage);

		double processingTime = (endTime - startTime)/(cvGetTickFrequency() * 1000.0);
		std::cout << k << " >> processing time : " << processingTime << " ms, error : " << error << std::endl;

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'i':
		case 'I':
			k++;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	cvReleaseImage(&resultImage);
	cvDestroyAllWindows();
}