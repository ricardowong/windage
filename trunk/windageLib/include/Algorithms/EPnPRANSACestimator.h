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
 * @file	EPnPRANSACestimator.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.09
 * @brief	It is implemetation of camera pose estimator class to use EPnP & RANSAC techniq
 */

#ifndef _EPNP_RANSAC_ESTIMATOR_H_
#define _EPNP_RANSAC_ESTIMATOR_H_

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
		 * @brief	class for camera pose estimation in 3D using EPnP & RANSAC
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT EPnPRANSACestimator : public PoseEstimator
		{
		protected:
			double confidence;
			int maxIteration;
		public:
			virtual char* GetFunctionName(){return "EPnPRANSACestimator";};
			EPnPRANSACestimator() : PoseEstimator()
			{
				this->reprojectionError = 2.0;
				this->confidence = 0.995;
				this->maxIteration = 1000;
			}
			virtual ~EPnPRANSACestimator()
			{
			}

			inline void SetMaxIteration(int iteration){this->maxIteration = iteration;};
			inline void SetConfidence(double confidence){this->confidence = confidence;};
			inline int GetMaxIteration(){return this->maxIteration;};
			inline double GetConfidence(){return this->confidence;};

			/**
			 * @fn	Calculate
			 * @brief
			 *		Implimentation function to calculate camera pose using attatched pair set of input feature points and reference feature points
			 * @remark
			 *		update the homography member value
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 *		the referencePoints is to be not planar
			 * @return
			 *		success or failure
			 */
			bool Calculate();
		};
		/** @} */ // addtogroup AlgorithmsPoseEstimator
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _EPNP_RANSAC_ESTIMATOR_H_