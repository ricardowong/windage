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
 * @file	SearchTree.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class for feature matching to use searching tree
 */

#ifndef _SEARCH_TREE_H_
#define _SEARCH_TREE_H_

#include <vector>

#include <cv.h>
#include "base.h"

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
		 * @defgroup AlgorithmsSearchTree Feature Matching
		 * @brief
				feature matching algorithm classes
		 * @addtogroup AlgorithmsSearchTree
		 * @{
		 */

		/**
		 * @brief	Abstract class for feature matching to use searching tree
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT SearchTree
		{
		protected:
			int DESCRIPTOR_DATA_TYPE;			///< descriptor data type CV_32F / CV_64F
			double nearestNeighbourhoodRatio;	///< ratio for nearest neigh bourhood : 0.7

		public:
			virtual char* GetFunctionName(){return "SearchTree";};
			SearchTree()
			{
				this->DESCRIPTOR_DATA_TYPE = CV_32F;
				this->nearestNeighbourhoodRatio = 0.7;
			}
			virtual ~SearchTree()
			{
			}

			inline void SetRatio(double ratio){this->nearestNeighbourhoodRatio = ratio;};
			inline double GetRatio(){return this->nearestNeighbourhoodRatio;};

			/**
			 * @fn	Training
			 * @brief
			 *		abstract virtual function to generate descriptor tree
			 * @remark
			 *		the result is each implementation class member storage
			 * @warning
			 *		pointList is not null
			 * @return
			 *		success or failure
			 */
			virtual bool Training(
								  std::vector<windage::FeaturePoint>* pointList	///< feature point list to generate tree
								  ) = 0;

			/**
			 * @fn	Matching
			 * @brief
			 *		abstract virtual function to match between input feature point and reference feature points
			 * @remark
			 *		the comparison is each implementation class member storage
			 * @warning
			 *		if difference is NULL pointer that not return the difference value
			 * @return
			 *		matched reference feature index
			 *		reference parameter difference is distance between input feature and matched refernece point
			 */
			virtual int Matching(
								 windage::FeaturePoint point,	///< input feature
								 double* difference = NULL		///< output reference pointer
								 ) = 0;
		};
		/** @} */ // addtogroup AlgorithmsSearchTree
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _SEARCH_TREE_H_