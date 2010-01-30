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

#ifndef _HOMOGRAPHY_ESTIMATOR_H_
#define _HOMOGRAPHY_ESTIMATOR_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Matrix.h"
#include "Structures/FeaturePoint.h"

namespace windage
{
	namespace Algorithms
	{
		class DLLEXPORT HomographyEstimator
		{
		protected:
			windage::Matrix3 homography;
			double reprojectionError;

			std::vector<windage::FeaturePoint>* referencePoints;
			std::vector<windage::FeaturePoint>* scenePoints;
			
		public:
			HomographyEstimator()
			{
				this->homography.m[0][0] = 1.0; this->homography.m[0][1] = 0.0; this->homography.m[0][2] = 0.0;
				this->homography.m[1][0] = 0.0; this->homography.m[1][1] = 1.0; this->homography.m[1][2] = 0.0;
				this->homography.m[2][0] = 0.0; this->homography.m[2][1] = 0.0; this->homography.m[2][2] = 1.0;
				reprojectionError = 2.0;

				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}
			virtual ~HomographyEstimator()
			{
				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}

			inline windage::Matrix3 GetHomography(){return this->homography;};
			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};
			inline void AttatchReferencePoint(std::vector<windage::FeaturePoint>* referencePoints){this->referencePoints = referencePoints;};
			inline void AttatchScenePoint(std::vector<windage::FeaturePoint>* scenePoints){this->scenePoints = scenePoints;};
			inline std::vector<windage::FeaturePoint>* GetReferencePoint(){return this->referencePoints;};
			inline std::vector<windage::FeaturePoint>* GetScenePoint(){return this->scenePoints;};

			windage::Vector3 ConvertObjectToImage(windage::Vector3 point)
			{
				return this->homography * point;
			}
			windage::Vector3 ConvertImageToObject(windage::Vector3 point)
			{
				return this->homography.Inverse() * point;
			}

			virtual bool Calculate() = 0;
		};
	}
}

#endif
