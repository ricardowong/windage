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

#include "FindPROSACHomography.h"

#include <algorithm>
#include <vector>
#include <time.h>

using namespace windage;
bool FindPROSACHomography::Calculate()
{
	CvMat _h = cvMat(3, 3, CV_32F, homography);

	std::vector<CvPoint2D32f> samplingObject; samplingObject.resize(4);
	std::vector<CvPoint2D32f> samplingReference; samplingReference.resize(4);
	
	CvMat samplingObjectPoints = cvMat(1, 4, CV_32FC2, &(samplingObject[0]));
	CvMat samplingReferencePoints = cvMat(1, 4, CV_32FC2, &(samplingReference[0]));

	// sort
	sort(matchedPoints->begin(), matchedPoints->end(), CompareDistanceLess());
	int count = matchedPoints->size();

	bool isProcessing = true;
	for(int i=0; i<maxIteration&&isProcessing; i++)
	{
		// reset
		for(int j=0; j<matchedPoints->size(); j++)
		{
			(*matchedPoints)[j].isInlier = false;
		}

		// sampling
		for(int j=0; j<4; j++)
		{
			int index = 0;
			if(i > count)
			{
				srand(time(NULL));
				index = rand() % count;
			}
			else
			{
				index= (i+j)%count;
			}
			samplingObject[j].x = (*matchedPoints)[index].pointScene.x;
			samplingObject[j].y = (*matchedPoints)[index].pointScene.y;

			samplingReference[j].x = (*matchedPoints)[index].pointReference.x;
			samplingReference[j].y = (*matchedPoints)[index].pointReference.y;
		}

		// calculate homograpy
		cvFindHomography(&samplingReferencePoints, &samplingObjectPoints, &_h);

		int inlinerCount = 0;
		// calculate consensus set
		for(int j=0; j<matchedPoints->size(); j++)
		{
			float pointX = (*matchedPoints)[j].pointReference.x;
			float pointY = (*matchedPoints)[j].pointReference.y;
			float pointZ = 1.0;

			float projectionPointX = homography[0] * pointX + homography[1] * pointY + homography[2] * pointZ;
			float projectionPointY = homography[3] * pointX + homography[4] * pointY + homography[5] * pointZ;
			float projectionPointZ = homography[6] * pointX + homography[7] * pointY + homography[8] * pointZ;
			projectionPointX /= projectionPointZ;
			projectionPointY /= projectionPointZ;

			float dx = abs((*matchedPoints)[j].pointScene.x - projectionPointX);
			float dy = abs((*matchedPoints)[j].pointScene.y - projectionPointY);
			if(dx+dy < this->reprojectionThreshold*2)
			{
				(*matchedPoints)[j].isInlier = true;
				inlinerCount++;
			}
		}

		// calculate 
		if( (float)inlinerCount / (float)count > this->terminationRatio)
		{
			std::vector<CvPoint2D32f> consensusObject;
			std::vector<CvPoint2D32f> consensusReference;

			for(int j=0; j<matchedPoints->size(); j++)
			{
				if((*matchedPoints)[j].isInlier)
				{
					consensusObject.push_back(cvPoint2D32f((*matchedPoints)[j].pointScene.x, (*matchedPoints)[j].pointScene.y));
					consensusReference.push_back(cvPoint2D32f((*matchedPoints)[j].pointReference.x, (*matchedPoints)[j].pointReference.y));
				}
			}
			CvMat consensusObjectPoints = cvMat(1, consensusObject.size(), CV_32FC2, &(consensusObject[0]));
			CvMat consensusReferencePoints = cvMat(1, consensusReference.size(), CV_32FC2, &(consensusReference[0]));

			cvFindHomography(&consensusReferencePoints, &consensusObjectPoints, &_h);
			return true;
		}
	}

	return false;
}