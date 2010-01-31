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

#ifndef _POSE_ESTIMATOR_H_
#define _POSE_ESTIMATOR_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Matrix.h"
#include "Structures/FeaturePoint.h"
#include "Structures/Calibration.h"

namespace windage
{
	namespace Algorithms
	{
		class DLLEXPORT PoseEstimator
		{
		protected:
			windage::Calibration* cameraParameter;
			double reprojectionError;

			std::vector<windage::FeaturePoint>* referencePoints;
			std::vector<windage::FeaturePoint>* scenePoints;
			
		public:
			PoseEstimator()
			{
				cameraParameter = NULL;
				reprojectionError = 2.0;

				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}
			virtual ~PoseEstimator()
			{
				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}

			inline void AttatchCameraParameter(windage::Calibration* cameraParameter){this->cameraParameter = cameraParameter;};
			inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};

			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};
			
			inline void AttatchReferencePoint(std::vector<windage::FeaturePoint>* referencePoints){this->referencePoints = referencePoints;};
			inline void AttatchScenePoint(std::vector<windage::FeaturePoint>* scenePoints){this->scenePoints = scenePoints;};
			inline std::vector<windage::FeaturePoint>* GetReferencePoint(){return this->referencePoints;};
			inline std::vector<windage::FeaturePoint>* GetScenePoint(){return this->scenePoints;};

			windage::Vector3 ConvertWorldToImage(windage::Vector3 point = windage::Vector3(0.0, 0.0, 0.0))
			{
				CvPoint imagePoint;
				if(cameraParameter)
				{
					imagePoint = cameraParameter->ConvertWorld2Image(point.x, point.y, point.z);
				}
				return windage::Vector3((double)imagePoint.x, (double)imagePoint.y, 1.0);
			}
			windage::Vector3 ConvertImageToWorld(windage::Vector3 point = windage::Vector3(0.0, 0.0, 1.0), double z = 0.0)
			{
				CvPoint2D64f worldPoint;
				if(cameraParameter)
				{
					worldPoint = cameraParameter->ConvertImage2World(point.x, point.y, z);
				}
				return windage::Vector3(worldPoint.x, worldPoint.y, z);
			}

			virtual bool Calculate() = 0;
		};
	}
}

#endif
