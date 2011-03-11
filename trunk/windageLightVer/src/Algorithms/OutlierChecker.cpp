/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
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

#include "Algorithms/OutlierChecker.h"
using namespace windage;
using namespace windage::Algorithms;

bool OutlierChecker::Calculate()
{
	if(this->homographyEstimator == NULL && this->poseEstimator == NULL)
		return false;

	if(this->homographyEstimator)
	{
		std::vector<windage::FeaturePoint>* refPoints = this->homographyEstimator->GetReferencePoint();
		std::vector<windage::FeaturePoint>* scePoints = this->homographyEstimator->GetScenePoint();

//		#pragma omp for
		for(int i=0; i<(int)refPoints->size(); i++)
		{
			windage::Vector3 imagePoint = this->homographyEstimator->ConvertObjectToImage((*refPoints)[i].GetPoint());
			imagePoint /= imagePoint.z;

			int index = 0;
			double error = (*scePoints)[i].GetPoint().getDistance(imagePoint);
			if(error <= this->reprojectionError)
			{
				(*refPoints)[i].SetOutlier(false);
				(*scePoints)[i].SetOutlier(false);
				index++;
			}
			else
			{
				(*refPoints)[i].SetOutlier(true);
				(*scePoints)[i].SetOutlier(true);
			}
		}
	}
	else
	{
		std::vector<windage::FeaturePoint>* refPoints = this->poseEstimator->GetReferencePoint();
		std::vector<windage::FeaturePoint>* scePoints = this->poseEstimator->GetScenePoint();

//		#pragma omp for
		for(int i=0; i<(int)refPoints->size(); i++)
		{
			windage::Vector3 imagePoint = this->poseEstimator->ConvertWorldToImage((*refPoints)[i].GetPoint());
			imagePoint /= imagePoint.z;

			int index = 0;
			double error = (*scePoints)[i].GetPoint().getDistance(imagePoint);
			if(error <= this->reprojectionError)
			{
				(*refPoints)[i].SetOutlier(false);
				(*scePoints)[i].SetOutlier(false);
				index++;
			}
			else
			{
				(*refPoints)[i].SetOutlier(true);
				(*scePoints)[i].SetOutlier(true);
			}
		}
	}

	return true;
}