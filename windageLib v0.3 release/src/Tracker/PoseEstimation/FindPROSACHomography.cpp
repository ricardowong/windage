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

#include "Tracker/PoseEstimation/FindPROSACHomography.h"

#include <algorithm>
#include <vector>
#include <time.h>

int PROSACUpdateNumIters(double p, double ep, int model_points, int max_iters)
{
	double num, denom;
	p = MAX(p, 0.);
    p = MIN(p, 1.);
    ep = MAX(ep, 0.);
    ep = MIN(ep, 1.);

    // avoid inf's & nan's
    num = MAX(1. - p, DBL_MIN);
    denom = 1. - pow(1. - ep,model_points);
	num = log(num);
    denom = log(denom);
    
	int result = denom >= 0 || -num >= max_iters*(-denom) ? max_iters : cvRound(num/denom);
	return result;
}


using namespace windage;
bool FindPROSACHomography::Calculate()
{
	double tickcount = (double)cvGetTickCount();

	float bestHomography[9];
	CvMat _bestH = cvMat(3, 3, CV_32F, bestHomography);
	CvMat _h = cvMat(3, 3, CV_32F, homography);

	std::vector<CvPoint2D32f> samplingObject;		samplingObject.resize(4);
	std::vector<CvPoint2D32f> samplingReference;	samplingReference.resize(4);

	int bestCount = 0;
	int count = matchedPoints->size();
	
	CvMat samplingObjectPoints = cvMat(1, 4, CV_32FC2, &(samplingObject[0]));
	CvMat samplingReferencePoints = cvMat(1, 4, CV_32FC2, &(samplingReference[0]));

	// sort
	srand((unsigned int)time(NULL));
	sort(matchedPoints->begin(), matchedPoints->end(), CompareDistanceLess());

	int samplingCount = 4;
	bool isProcessing = true;
	for(int i=0; i<maxIteration&&isProcessing; i++)
	{
		// reset
		for(int j=0; j<(int)matchedPoints->size(); j++)
		{
			(*matchedPoints)[j].isInlier = false;
		}

		// sampling
		double Tn1 = (double)samplingCount;
		double Tn = Tn1 * (double)(count + 1) / (double)(count + 1 - samplingCount);
		samplingCount = samplingCount + (int)(Tn - Tn1 + 1.0);
		samplingCount = MIN(count-1, samplingCount);

		int index[4] = {-1, -1, -1, -1};
		for(int j=0; j<4; j++)
		{
			int tempIndex = rand() % samplingCount;
			while(index[0] == tempIndex || index[1] == tempIndex || index[2] == tempIndex)
			{
				tempIndex = rand() % samplingCount;						
			}
			index[j] = tempIndex;
		}

		for(int j=0; j<4; j++)
		{
			int tempIndex = index[j];

			samplingObject[j].x = (*matchedPoints)[tempIndex].pointScene.x;
			samplingObject[j].y = (*matchedPoints)[tempIndex].pointScene.y;

			samplingReference[j].x = (*matchedPoints)[tempIndex].pointReference.x;
			samplingReference[j].y = (*matchedPoints)[tempIndex].pointReference.y;
		}

		// calculate homograpy
		cvFindHomography(&samplingReferencePoints, &samplingObjectPoints, &_h);

		int inlinerCount = 0;
		// calculate consensus set
		for(int j=0; j<(int)matchedPoints->size(); j++)
		{
			float error = ComputeReprojError((*matchedPoints)[j].pointReference, (*matchedPoints)[j].pointScene, homography);
			if(error < this->reprojectionThreshold)
			{
				(*matchedPoints)[j].isInlier = true;
				inlinerCount++;
			}
		}

		if(inlinerCount > bestCount)
		{
			bestCount = inlinerCount;
			for(int k=0; k<9; k++) bestHomography[k] = homography[k];
			maxIteration = PROSACUpdateNumIters(confidence, (double)(count - inlinerCount)/count, 4, maxIteration);
		}


		// find time-out
		if((cvGetTickCount() - tickcount) > this->timeout)
		{
			return false;
		}
	}

	// terminate
	if(bestCount > 4)
	{
		for(int j=0; j<(int)matchedPoints->size(); j++)
		{
			float error = ComputeReprojError((*matchedPoints)[j].pointReference, (*matchedPoints)[j].pointScene, bestHomography);
			if(error < this->reprojectionThreshold)
			{
				(*matchedPoints)[j].isInlier = true;
			}
			else
			{
				(*matchedPoints)[j].isInlier = false;
			}
		}

		std::vector<CvPoint2D32f> consensusObject;		consensusObject.resize(bestCount);
		std::vector<CvPoint2D32f> consensusReference;	consensusReference.resize(bestCount);

		int index = 0;
		for(int j=0; j<(int)matchedPoints->size(); j++)
		{
			if((*matchedPoints)[j].isInlier)
			{
				consensusObject[index]=cvPoint2D32f((*matchedPoints)[j].pointScene.x, (*matchedPoints)[j].pointScene.y);
				consensusReference[index]=cvPoint2D32f((*matchedPoints)[j].pointReference.x, (*matchedPoints)[j].pointReference.y);
				index++;
			}
		}
		CvMat consensusObjectPoints = cvMat(1, consensusObject.size(), CV_32FC2, &(consensusObject[0]));
		CvMat consensusReferencePoints = cvMat(1, consensusReference.size(), CV_32FC2, &(consensusReference[0]));
		cvFindHomography(&consensusReferencePoints, &consensusObjectPoints, &_h);
		return true;
	}

	return false;
}