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
#include "../Algorithms/InverseCompositional.h"

const int IMAGE_SEQ_COUNT = 200;
const char* IMAGE_SEQ_FILE_NAME = "seq/im%03d.pgm";

const int GAUSSIAN_BLUR = 5;

const int TEMPLATE_WIDTH = 100;
const int TEMPLATE_HEIGHT = 100;
const double HOMOGRAPHY_DELTA = 0.01;
const int MAX_ITERATION = 80;

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

	cvLine(image, cvPoint(outPoint1.x, outPoint1.y), cvPoint(outPoint2.x, outPoint2.y), color, thickness);
	cvLine(image, cvPoint(outPoint2.x, outPoint2.y), cvPoint(outPoint3.x, outPoint3.y), color, thickness);
	cvLine(image, cvPoint(outPoint3.x, outPoint3.y), cvPoint(outPoint4.x, outPoint4.y), color, thickness);
	cvLine(image, cvPoint(outPoint4.x, outPoint4.y), cvPoint(outPoint1.x, outPoint1.y), color, thickness);
}

void  main()
{
	char message[100];
	cvNamedWindow("template");
	cvNamedWindow("samplingESM");
	cvNamedWindow("samplingIC");
	cvNamedWindow("result");

	// initialize
	sprintf(message, IMAGE_SEQ_FILE_NAME, 0);
	IplImage* saveImage = cvLoadImage(message, 0);
	cvSmooth(saveImage, saveImage, CV_GAUSSIAN, GAUSSIAN_BLUR, GAUSSIAN_BLUR);

	int width = saveImage->width;
	int height = saveImage->height;
	int startX = (width-TEMPLATE_WIDTH)/2;
	int startY = (height-TEMPLATE_HEIGHT)/2;
	double q = TEMPLATE_WIDTH * TEMPLATE_HEIGHT;
	
	IplImage* inputImage = NULL;
	IplImage* resultImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage* templateImage = cvCreateImage(cvSize(TEMPLATE_WIDTH, TEMPLATE_HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* samplingESMImage = NULL;
	IplImage* samplingICImage = NULL;

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
	windage::Matrix3 homographyESM = e;
	windage::Matrix3 homographyIC = e;

	// Template based Tracking using Homography ESM
	windage::HomographyESM* esm = new windage::HomographyESM(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	esm->AttatchTemplateImage(templateImage);
	esm->SetInitialHomography(e);
	esm->Initialize();

	// Template based Tracking using Inverse Compositional
	windage::InverseCompositional* ic = new windage::InverseCompositional(TEMPLATE_WIDTH, TEMPLATE_HEIGHT);
	ic->AttatchTemplateImage(templateImage);
	ic->SetInitialHomography(e);
	ic->Initialize();

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
		cvSmooth(inputImage, inputImage, CV_GAUSSIAN, GAUSSIAN_BLUR, GAUSSIAN_BLUR);

		cvCvtColor(inputImage, resultImage, CV_GRAY2BGR);

		// processing
		int64 startTimeESM = cvGetTickCount();
		double errorESM = 0.0;
		int iterESM = 0;
		for(iterESM=0; iterESM<MAX_ITERATION; iterESM++)
		{
			double delta = 1.0;
			errorESM = esm->UpdateHomography(inputImage, &delta);
			homographyESM = esm->GetHomography();
			samplingESMImage = esm->GetSamplingImage();

			if(delta < HOMOGRAPHY_DELTA)
				break;			
		}
		int64 endTimeESM = cvGetTickCount();

		int64 startTimeIC = cvGetTickCount();
		double errorIC = 0.0;
		int iterIC = 0;
		for(iterIC=0; iterIC<MAX_ITERATION; iterIC++)
		{
			double delta = 1.0;
			errorIC = ic->UpdateHomography(inputImage, &delta);
			homographyIC = ic->GetHomography();
			samplingICImage = ic->GetSamplingImage();

			if(delta < HOMOGRAPHY_DELTA)
				break;
		}
		int64 endTimeIC = cvGetTickCount();

		// draw result
		DrawResult(resultImage, homographyESM, CV_RGB(255, 0, 0), 3);
		DrawResult(resultImage, homographyIC, CV_RGB(0, 255, 0), 3);

		char message[500];
		double processingTimeESM = (endTimeESM - startTimeESM)/(cvGetTickFrequency() * 1000.0);
		double processingTimeIC = (endTimeIC - startTimeIC)/(cvGetTickFrequency() * 1000.0);
		sprintf(message, "ESM (error : %.2lf, %d iter, %.2lf ms), IC (error : %.2lf, %d iter, %.2lf ms)",
			errorESM, iterESM, processingTimeESM, errorIC, iterIC, processingTimeIC);
		std::cout << message << std::endl;

		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 60), message, 0.5);
		sprintf(message, "press 'I' key to get next image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), message, 0.5);
		sprintf(message, "red line is ESM algorithm & green line is Inverse Compositional algorithm");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 40), message, 0.5 );
 		
		// draw image
		cvShowImage("samplingESM", samplingESMImage );
		cvShowImage("samplingIC", samplingICImage);
		cvShowImage("result", resultImage);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case ' ':
			cvWaitKey(0);
			break;
		case 'i':
		case 'I':
			k++;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
		k++;
	}

	cvReleaseImage(&resultImage);
	cvDestroyAllWindows();
}