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

/**
 * @file	OutlierChecker.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is implemetation to check whether outlier or not
 */

#ifndef _OUTLIER_CHECKER_H_
#define _OUTLIER_CHECKER_H_

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
		 * @brief	class to checke whether outlier or not base on homography estimator
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT OutlierChecker
		{
		private:
			HomographyEstimator* homographyEstimator;	///< homography Estimator to attatch reference pointer at out-side
			PoseEstimator* poseEstimator;
			double reprojectionError;					///< threshold to determin outlier or not

		public:
			virtual char* GetFunctionName(){return "OutlierChecker";};
			OutlierChecker()
			{
				homographyEstimator = NULL;
				reprojectionError = 2.0;
			}
			~OutlierChecker()
			{
			}

			/**
			 * @fn	AttatchEstimator
			 * @brief
			 *		attatch homograpy estimator to member pointer from out-side
			 * @remark
			 *		reference & scene points in homograpy estimator will be updated after calling the calculate function
			 * @warning
			 *		homograpy estimator is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchEstimator(HomographyEstimator* homographyEstimator){this->homographyEstimator = homographyEstimator; this->poseEstimator = NULL;};
			inline void AttatchEstimator(PoseEstimator* poseEstimator){this->poseEstimator = poseEstimator; this->homographyEstimator = NULL;};

			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};

			/**
			 * @fn	Calculate
			 * @brief
			 *		calcuation function to determin wheter outlier or not
			 *		using attatched pair set of input feature points and reference feature points in homograpy estimator
			 * @return
			 *		success or failure
			 */
			bool Calculate();
		};
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _OUTLIER_CHECKER_H_