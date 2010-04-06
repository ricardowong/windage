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
 * @file	LMmethod.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is implemetation of homography refinement class to use Levenberg-Marqardt method
 */

#ifndef _LM_METHOD_REFINER_H_
#define _LM_METHOD_REFINER_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Matrix.h"
#include "Structures/FeaturePoint.h"
#include "Algorithms/HomographyRefiner.h"

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
		 * @brief	class for homography refinement class to use LM method
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT LMmethod : public HomographyRefiner
		{
		protected:
		public:
			virtual char* GetFunctionName(){return "LMmethod";};
			LMmethod() : HomographyRefiner()
			{
			}
			~LMmethod()
			{
			}

			/**
			 * @fn	Calculate
			 * @brief
			 *		Implimentation function to refinement homography using attatched initialized homography and pair set of input feature points and reference feature points
			 * @warning
			 *		the nubmer of referencePoints and the number of scenePoints is to be same
			 * @return
			 *		success or failure
			 */
			bool Calculate();
		};
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _LM_METHOD_REFINER_H_