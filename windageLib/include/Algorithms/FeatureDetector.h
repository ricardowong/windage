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
 * @file	FeatureDetector.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class for detecting feature algorithms
 */

#ifndef _FEATURE_DETECTOR_H_
#define _FEATURE_DETECTOR_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Vector.h"
#include "Structures/FeaturePoint.h"

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
		 * @defgroup AlgorithmsFeatureDetector Feature Detector
		 * @brief
				feature detector algorithm classes
		 * @addtogroup AlgorithmsFeatureDetector
		 * @{
		 */

		/**
		 * @brief	Abstract class for feature detection
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT FeatureDetector
		{
		protected:
			std::vector<windage::FeaturePoint> keypoints;	///< repository for feature poitns
			double threshold;

		public:
			virtual char* GetFunctionName(){return "FeatureDetector";};
			FeatureDetector()
			{
			}
			virtual ~FeatureDetector()
			{
			}

			inline int GetKeypointsCount(){return (int)this->keypoints.size();};
			inline std::vector<windage::FeaturePoint>* GetKeypoints(){return &this->keypoints;};

			/**
			 * @fn	DoExtractKeypointsDescriptor
			 * @brief
			 *		abstract virtual function for extract keypoints & generate descriptors
			 * @remark
			 *		the result is depend on threshold (member valuable)
			 * @warning
			 *		input image is always gray image (1-channel)
			 * @return
			 *		success or failure
			 */
			virtual bool DoExtractKeypointsDescriptor(
													  IplImage* grayImage	///< input image
													  ) = 0;
			
			inline void SetThreshold(double threshold){if(threshold > 0)this->threshold = threshold;};
			inline double GetThreshold(){return this->threshold;};

			/**
			 * @fn	DrawKeypoint
			 * @brief
			 *		draw keypoint information
			 * @remark
			 *		the result is refer to parameter keypoint
			 * @warning
			 *		input image is recommended as color image (3-channel)
			 */
			void DrawKeypoint(IplImage* colorImage, FeaturePoint point, CvScalar color = CV_RGB(255, 0, 0));

			/**
			 * @fn	DrawKeypoints
			 * @brief
			 *		draw keypoints information
			 * @remark
			 *		the result is refer to member value that is keypoints
			 * @warning
			 *		input image is recommended as color image (3-channel)
			 */
			void DrawKeypoints(IplImage* colorImage, CvScalar color = CV_RGB(255, 0, 0));
		};
		/** @} */ // addtogroup AlgorithmsFeatureDetector
		/** @} */ // addtogroup Algorithms
	}
}

#endif // _FEATURE_DETECTOR_H_
