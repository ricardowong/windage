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
 * @file	HomographyEstimator.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class for camera pose estimator using Homography
 */

#ifndef _HOMOGRAPHY_ESTIMATOR_H_
#define _HOMOGRAPHY_ESTIMATOR_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Matrix.h"
#include "Structures/FeaturePoint.h"
#include "Algorithms/PoseEstimator.h"

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
		 * @defgroup AlgorithmsHomography Homography Estimator
		 * @brief
				camera pose estimator in 2D using homography
		 * @addtogroup AlgorithmsHomography
		 * @{
		 */

		/**
		 * @brief	Abstract class for homography estimation in 2D
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT HomographyEstimator : public PoseEstimator
		{
		protected:
			windage::Matrix3 homography;	///< homography parameter to update from calculate function
			
		public:
			virtual char* GetFunctionName(){return "HomographyEstimator";};
			HomographyEstimator() : PoseEstimator()
			{
				this->homography.m[0][0] = 1.0; this->homography.m[0][1] = 0.0; this->homography.m[0][2] = 0.0;
				this->homography.m[1][0] = 0.0; this->homography.m[1][1] = 1.0; this->homography.m[1][2] = 0.0;
				this->homography.m[2][0] = 0.0; this->homography.m[2][1] = 0.0; this->homography.m[2][2] = 1.0;
			}
			virtual ~HomographyEstimator()
			{
			}

			inline windage::Matrix3* GetHomography(){return &this->homography;};
			
			windage::Vector3 ConvertObjectToImage(windage::Vector3 point = windage::Vector3(0.0, 0.0, 1.0))
			{
				return this->homography * point;
			}
			windage::Vector3 ConvertImageToObject(windage::Vector3 point = windage::Vector3(0.0, 0.0, 1.0))
			{
				return this->homography.Inverse() * point;
			}

			/**
			 * @fn	Calculate
			 * @brief
			 *		abstract virtual function to calculate homograpy using attatched pair set of input feature points and reference feature points
			 * @remark
			 *		update the homography member value
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 * @return
			 *		success or failure
			 */
			virtual bool Calculate() = 0;

			/**
			 * @fn	DecomposeHomography
			 * @brief
			 *		Decompose homography to camera calibration using intrinsic matrix
			 * @remark
			 *		if you want to change to camera calibration to homography
			 * @warning
			 *		if the input parameter that is camrea calibration parameter is NULL than not decompose the pose and return false
			 * @return
			 *		success or failure
			 */
			bool DecomposeHomography(windage::Calibration* cameraParameter=NULL);
		};
		/** @} */ // addtogroup AlgorithmsHomographyEstimator
		/** @} */ // addtogroup AlgorithmsPoseEstimator
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _HOMOGRAPHY_ESTIMATOR_H_