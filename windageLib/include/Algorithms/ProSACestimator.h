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
 * @file	ProSACestimator.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is implemetation of homography estimation class to use Propagation SAmpling Consensus techniq
 */

#ifndef _PROSAC_ESTIMATOR_H_
#define _PROSAC_ESTIMATOR_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Algorithms/HomographyEstimator.h"

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
		 * @defgroup AlgorithmsHomography camera pose estimator in 2D
		 * @brief
				camera pose estimator in 2D
		 * @addtogroup AlgorithmsHomography
		 * @{
		 */

		/**
		 * @brief	class for homography estimation to use ProSAC techniq
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT ProSACestimator : public HomographyEstimator
		{
		private:
			double confidence;
			int maxIteration;
		public:
			virtual char* GetFunctionName(){return "ProSACestimator";};
			ProSACestimator() : HomographyEstimator()
			{
				this->reprojectionError = 2.0;
				this->confidence = 0.995;
				this->maxIteration = 1000;
			}
			~ProSACestimator()
			{
			}

			inline void SetMaxIteration(int iteration){this->maxIteration = iteration;};
			inline void SetConfidence(double confidence){this->confidence = confidence;};

			/**
			 * @fn	Calculate
			 * @brief
			 *		Implimentation function to calculate homograpy using attatched pair set of input feature points and reference feature points
			 * @remark
			 *		update the homography member value
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 * @return
			 *		success or failure
			 */
			bool Calculate();
		};

		/**
		 * @brief	matched point class for ProSAC techniq
		 *			It is support as STL sort function
		 * @author	Woonhyuk Baek
		 */
		class MatchedPoint
		{
		public:
			CvPoint2D64f pointScene;
			CvPoint2D64f pointReference;
			double distance;
			bool isInlier;

			MatchedPoint(CvPoint2D64f pointScene, CvPoint2D64f pointReference, double distance = 0.0)
			{
				this->pointScene = pointScene;
				this->pointReference = pointReference;

				this->distance = distance;
				isInlier = false;
			}

			void operator=(class MatchedPoint oprd)
			{
				this->pointScene = oprd.pointScene;
				this->pointReference = oprd.pointReference;
				this->distance = oprd.distance;
				this->isInlier = oprd.isInlier;
			}

			bool operator==(class MatchedPoint oprd)
			{
				return this->distance == oprd.distance;
			}
			bool operator<(class MatchedPoint &oprd)
			{
				return this->distance < oprd.distance;
			}
		};

		/**
		 * @brief	Compair distance function for MatchedPoint
		 *			It is for support as STL sort function
		 * @author	Woonhyuk Baek
		 */
		typedef struct _CompareDistanceLess
		{
			bool operator()(const windage::Algorithms::MatchedPoint& p, const windage::Algorithms::MatchedPoint& q) const
			{
				return p.distance < q.distance;
			}
		}CompareDistanceLess;

		/** @} */ // addtogroup AlgorithmsHomographyEstimator
		/** @} */ // addtogroup AlgorithmsPoseEstimator
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _PROSAC_ESTIMATOR_H_