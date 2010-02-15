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

#include "Algorithms/ProSACestimator.h"
using namespace windage;
using namespace windage::Algorithms;

double ComputeReprojError(CvPoint2D64f refPoint, CvPoint2D64f scePoint, double* homography)
{
	double ww = 1./(homography[6] * refPoint.x + homography[7] * refPoint.y + homography[8]);
	double x = (homography[0] * refPoint.x + homography[1] * refPoint.y +  homography[2]) * ww;
	double y = (homography[3] * refPoint.x + homography[4] * refPoint.y +  homography[5]) * ww;
	double error = sqrt((scePoint.x - x)*(scePoint.x - x)) + sqrt((scePoint.y - y)*(scePoint.y - y));
	return error;
}

bool ProSACestimator::Calculate()
{
	if(referencePoints == NULL || scenePoints == NULL)
		return false;
	int n = (int)referencePoints->size();
	if(n != (int)scenePoints->size())
		return false;
	if(n < 4)
		return false;

	double h[9];
	double bestHomography[9];
	CvMat _h = cvMat(3, 3, CV_64F, h);
	
	std::vector<CvPoint2D64f> samplingObject;		samplingObject.resize(4);
	std::vector<CvPoint2D64f> samplingReference;	samplingReference.resize(4);
	CvMat samplingObjectPoints = cvMat(1, 4, CV_64FC2, &(samplingObject[0]));
	CvMat samplingReferencePoints = cvMat(1, 4, CV_64FC2, &(samplingReference[0]));

	CvRNG rng = cvRNG(cvGetTickCount());
	int bestCount = 0;
	int count = (int)this->referencePoints->size();

	// copy matching point information
	std::vector<windage::Algorithms::MatchedPoint> matchedPoints;
	for(int i=0; i<count; i++)
	{
		matchedPoints.push_back(
			windage::Algorithms::MatchedPoint
			(
				cvPoint2D64f((*this->scenePoints)[i].GetPoint().x, (*this->scenePoints)[i].GetPoint().y),
				cvPoint2D64f((*this->referencePoints)[i].GetPoint().x, (*this->referencePoints)[i].GetPoint().y),
				(*this->referencePoints)[i].GetDistance()
			)
		);
	}

	// sort
	sort(matchedPoints.begin(), matchedPoints.end(), CompareDistanceLess());

	int samplingCount = 4;
	int maxIter = this->maxIteration;
	for(int i=0; i<maxIter; i++)
	{
		// reset
		for(int j=0; j<count; j++)
		{
			matchedPoints[j].isInlier = false;
		}

		// sampling
		double Tn1 = (double)samplingCount;
		double Tn = Tn1 * (double)(count + 1) / (double)(count + 1 - samplingCount);
		samplingCount = samplingCount + (int)(Tn - Tn1 + 1.0);
		samplingCount = MIN(count-1, samplingCount);

		int index[4] = {-1, -1, -1, -1};
		for(int j=0; j<4; j++)
		{
			int tempIndex = cvRandInt(&rng) % samplingCount;
			while(index[0] == tempIndex || index[1] == tempIndex || index[2] == tempIndex)
			{
				tempIndex = cvRandInt(&rng) % samplingCount;						
			}
			index[j] = tempIndex;
		}

		for(int j=0; j<4; j++)
		{
			int tempIndex = index[j];

			samplingObject[j].x = matchedPoints[tempIndex].pointScene.x;
			samplingObject[j].y = matchedPoints[tempIndex].pointScene.y;

			samplingReference[j].x = matchedPoints[tempIndex].pointReference.x;
			samplingReference[j].y = matchedPoints[tempIndex].pointReference.y;
		}

		// calculate homograpy
		cvFindHomography(&samplingReferencePoints, &samplingObjectPoints, &_h);

		int inlinerCount = 0;
		// calculate consensus set
		for(int j=0; j<count; j++)
		{
			double error = ComputeReprojError(matchedPoints[j].pointReference, matchedPoints[j].pointScene, h);
			if(error < this->reprojectionError)
			{
				matchedPoints[j].isInlier = true;
				inlinerCount++;
			}
		}

		if(inlinerCount > bestCount)
		{
			bestCount = inlinerCount;
			for(int k=0; k<9; k++)
				bestHomography[k] = h[k];

			if(this->confidence > 0)
				maxIter = this->RANSACUpdateNumIters(this->confidence, (double)(count - inlinerCount)/(double)count, 4, maxIteration);
		}
	}

	// terminate
	if(bestCount >= 4)
	{
		for(int j=0; j<count; j++)
		{
			double error = ComputeReprojError(matchedPoints[j].pointReference, matchedPoints[j].pointScene, bestHomography);
			if(error < this->reprojectionError)
			{
				matchedPoints[j].isInlier = true;
			}
			else
			{
				matchedPoints[j].isInlier = false;
			}
		}

		std::vector<CvPoint2D64f> consensusReference;
		std::vector<CvPoint2D64f> consensusObject;		

		for(int j=0; j<count; j++)
		{
			if(matchedPoints[j].isInlier)
			{
				consensusReference.push_back(cvPoint2D64f(matchedPoints[j].pointReference.x, matchedPoints[j].pointReference.y));
				consensusObject.push_back(cvPoint2D64f(matchedPoints[j].pointScene.x, matchedPoints[j].pointScene.y));
			}
		}

		CvMat consensusReferencePoints = cvMat(1, (int)consensusReference.size(), CV_64FC2, &(consensusReference[0]));
		CvMat consensusObjectPoints = cvMat(1, (int)consensusObject.size(), CV_64FC2, &(consensusObject[0]));
		
		cvFindHomography(&consensusReferencePoints, &consensusObjectPoints, &_h);

		// update
		for(int i=0; i<9; i++)
			this->homography.m1[i] = h[i];

		this->DecomposeHomography(this->cameraParameter);

		return true;
	}

	return false;
}