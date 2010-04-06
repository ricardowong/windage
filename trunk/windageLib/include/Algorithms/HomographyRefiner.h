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
 * @file	HomgraphyRefiner.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class to refine homography
 */

#ifndef _HOMOGRAPHY_REFINER_H_
#define _HOMOGRAPHY_REFINER_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Matrix.h"
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
		 * @brief	Abstract class to refine homography
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT HomographyRefiner
		{
		protected:
			windage::Matrix3* homography;	///< homography parameter to attatch reference pointer at out-side
			int maxIteration;				///< the number of refinement iteration

			/** the nubmer of referencePoints and the number of scenePoints is to be same */
			std::vector<windage::FeaturePoint>* referencePoints;	// reference points to attatch reference pointer at out-side
			std::vector<windage::FeaturePoint>* scenePoints;		// reference points to attatch reference pointer at out-side
			
		public:
			virtual char* GetFunctionName(){return "HomographyRefiner";};
			HomographyRefiner()
			{
				homography = NULL;
				maxIteration = 10;

				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}
			virtual ~HomographyRefiner()
			{
				this->referencePoints = NULL;
				this->scenePoints = NULL;
			}

			/**
			 * @fn	AttatchHomography
			 * @brief
			 *		attatch homography parameter to member pointer from out-side
			 * @remark
			 *		homography parameter will be updated after calling the calculate function
			 * @warning
			 *		homography parameter is to be initialized using homography estimator
			 *		homography parameter is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchHomography(windage::Matrix3* homography){this->homography = homography;};
			inline windage::Matrix3* GetHomography(){return this->homography;};

			inline void SetMaxIteration(int iteration){this->maxIteration = iteration;};

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

			/**
			 * @fn	Calculate
			 * @brief
			 *		abstract virtual function to refinement homography using attatched initialized homography and pair set of input feature points and reference feature points
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 * @return
			 *		success or failure
			 */
			virtual bool Calculate() = 0;
		};
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _HOMOGRAPHY_REFINER_H_