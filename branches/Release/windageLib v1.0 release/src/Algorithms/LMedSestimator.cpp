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

#include "Algorithms/LMedSestimator.h"
using namespace windage;
using namespace windage::Algorithms;

bool LMedSestimator::Calculate()
{
	if(referencePoints == NULL || scenePoints == NULL)
		return false;
	int n = (int)referencePoints->size();
	if(n != (int)scenePoints->size())
		return false;
	if(n < 4)
		return false;

	const int HOMOGRAPHY_PARAM_COUNT = 9;
	float localHomography[HOMOGRAPHY_PARAM_COUNT];
	CvMat _h = cvMat(3, 3, CV_32F, localHomography);

	std::vector<CvPoint2D32f> refPoints; refPoints.resize(n);
	std::vector<CvPoint2D32f> scePoints; scePoints.resize(n);
	for(int i=0; i<n; i++)
	{
		windage::Vector3 ref = (*this->referencePoints)[i].GetPoint();
		windage::Vector3 sce = (*this->scenePoints)[i].GetPoint();

		refPoints[i] = cvPoint2D32f(ref.x, ref.y);
		scePoints[i] = cvPoint2D32f(sce.x, sce.y);
	}

	CvMat _refPoints = cvMat(1, n, CV_32FC2, &(refPoints[0]));
	CvMat _scePoints = cvMat(1, n, CV_32FC2, &(scePoints[0]));

	cvFindHomography(&_refPoints, &_scePoints, &_h, CV_LMEDS);

	for(int i=0; i<HOMOGRAPHY_PARAM_COUNT; i++)
		this->homography.m1[i] = (double)localHomography[i];

	this->DecomposeHomography(this->cameraParameter);

	return true;
}