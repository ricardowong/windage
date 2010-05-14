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

#include "Algorithms/OpticalFlow.h"
using namespace windage;
using namespace windage::Algorithms;

void OpticalFlow::Release()
{
	if(pyramid1) cvReleaseImage(&pyramid1);
	pyramid1 = NULL;
	if(pyramid2) cvReleaseImage(&pyramid2);
	pyramid2 = NULL;
}

void OpticalFlow::Initialize(int width, int height, CvSize windowSize, int pyramidLevel)
{
	this->Release();
	this->SetImageSize(width, height);

	this->SetWindowSize(windowSize);
	this->SetPyramidLevel(pyramidLevel);

	terminationCriteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);

	if(pyramid1) cvReleaseImage(&pyramid1);
	if(pyramid2) cvReleaseImage(&pyramid2);
	pyramid1 = cvCreateImage(this->GetImageSize(), IPL_DEPTH_8U, 1);
	pyramid2 = cvCreateImage(this->GetImageSize(), IPL_DEPTH_8U, 1);
}

int OpticalFlow::TrackFeatures(IplImage* prevGrayImage, IplImage* currGrayImage, std::vector<FeaturePoint>* prevPoints, std::vector<FeaturePoint>* currPoints)
{
	int pointCount = MIN((int)prevPoints->size(), this->MAX_POINT_COUNT);
	if(pointCount >= 1)
	{
		IplImage* tempPrev = cvCloneImage(prevGrayImage);
		IplImage* tempCurr = cvCloneImage(currGrayImage);

//		cvSmooth(tempPrev, tempPrev, CV_GAUSSIAN, 3, 3);
//		cvSmooth(tempCurr, tempCurr, CV_GAUSSIAN, 3, 3);

		for(int i=0; i<pointCount; i++)
			this->feature1[i] = cvPoint2D32f((*prevPoints)[i].GetPoint().x, (*prevPoints)[i].GetPoint().y);

		cvCalcOpticalFlowPyrLK(tempPrev, tempCurr, pyramid1, pyramid2, feature1, feature2, pointCount, this->windowSize, this->pyramidLevel, foundFeature, errorFeature, terminationCriteria, 0);

		cvReleaseImage(&tempPrev);
		cvReleaseImage(&tempCurr);

		int index = 0;
		for(int i=0; i<pointCount; i++)
		{
			if(foundFeature[i] != 0)
			{
				windage::FeaturePoint point = (*prevPoints)[i];
				point.SetPoint(windage::Vector3(feature2[i].x, feature2[i].y, 1.0));
				currPoints->push_back(point);
				index++;
			}
			else
			{
				(*prevPoints)[i].SetOutlier(true);
				currPoints->push_back((*prevPoints)[i]);
			}
		}
		return 1;
	}
	else
	{
		return 0;
	}
}

