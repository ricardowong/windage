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

/**
 * @file	PoseEstimator.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class for camera pose estimator
 */

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
		/**
		 * @defgroup Algorithms Algorithm classes
		 * @brief
		 *		algorithm classes
		 * @addtogroup Algorithms
		 * @{
		 */

		/**
		 * @defgroup AlgorithmsPoseEstimator Pose Estimator
		 * @brief
				camera pose estimator in 3D
		 * @addtogroup AlgorithmsPoseEstimator
		 * @{
		 */

		/**
		 * @brief	Abstract class for camera pose estimation in 3D
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT PoseEstimator
		{
		protected:
			windage::Calibration* cameraParameter;	///< camera calibration parameter to attatch reference pointer at out-side
			double reprojectionError;				///< threshold to determin outlier or not

			/** the nubmer of referencePoints and the number of scenePoints is to be same */
			std::vector<windage::FeaturePoint>* referencePoints;	///< reference feature pointers to attatch pointer at out-side
			std::vector<windage::FeaturePoint>* scenePoints;		///< scene feature pointers to attatch pointer at out-side
			
		public:
			virtual char* GetFunctionName(){return "PoseEstimator";};
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

			/**
			 * @fn	AttatchCameraParameter
			 * @brief
			 *		attatch camera parameter to member pointer from out-side
			 * @remark
			 *		camera parameter will be updated after calling the calculate function
			 * @warning
			 *		camera parameter is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchCameraParameter(windage::Calibration* cameraParameter){this->cameraParameter = cameraParameter;};

			/**
			 * @fn	AttatchReferencePoint
			 * @brief
			 *		attatch reference points to member pointer from out-side
			 * @warning
			 *		the reference points is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchReferencePoint(std::vector<windage::FeaturePoint>* referencePoints){this->referencePoints = referencePoints;};

			/**
			 * @fn	AttatchScenePoint
			 * @brief
			 *		attatch scene points to member pointer from out-side
			 * @warning
			 *		the scene points is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchScenePoint(std::vector<windage::FeaturePoint>* scenePoints){this->scenePoints = scenePoints;};

			inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};
			inline std::vector<windage::FeaturePoint>* GetReferencePoint(){return this->referencePoints;};
			inline std::vector<windage::FeaturePoint>* GetScenePoint(){return this->scenePoints;};
		
			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};


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

			/**
			 * @fn	Calculate
			 * @brief
			 *		abstract virtual function to calculate camera pose using attatched pair set of input feature points and reference feature points
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 * @return
			 *		success or failure
			 */
			virtual bool Calculate() = 0;

			/**
			 * @fn	RANSACUpdateNumIters
			 * @brief
			 *		to calculate RANSAC iteration number
			 * @return
			 *		max iteration numbers
			 */
			int RANSACUpdateNumIters(double p, double ep, int model_points, int max_iters)
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

		};
		/** @} */ // addtogroup AlgorithmsPoseEstimator
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _POSE_ESTIMATOR_H_