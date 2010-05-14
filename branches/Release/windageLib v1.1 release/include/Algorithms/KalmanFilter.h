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
 * @file	KalmanFilter.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.12
 * @brief	It is implemetation of kalman filter class to prevent zitering
 */

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Vector.h"
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
		 * @brief	class for kalman filtering to prevent zitering
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT KalmanFilter
		{
		private:
			/** Tx, Ty, Tz, dTx, dTy, dTz */
			CvKalman* kalman;
			CvMat* measurement;

		public:
			virtual char* GetFunctionName(){return "KalmanFilter";};
			KalmanFilter()
			{
				kalman = cvCreateKalman( 6, 3, 0 );
				measurement = cvCreateMat( 3, 1, CV_32FC1 );
				this->Initialize();
			}
			~KalmanFilter()
			{
				if(kalman) cvReleaseKalman(&kalman);
				if(measurement) cvReleaseMat(&measurement);
			}

			void Initialize();
			windage::Vector3 Predict();
			bool Correct(windage::Vector3 T);
		};
		/** @} */ // addtogroup AlgorithmsPoseEstimator
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _KALMAN_FILTER_H_