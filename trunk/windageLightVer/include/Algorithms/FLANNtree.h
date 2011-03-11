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
 * @file	FLANNtree.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is implemetation of search tree class to use FLANN-tree altorithm
 */

#ifndef _FLANN_TREE_H_
#define _FLANN_TREE_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/FeaturePoint.h"
#include "Algorithms/SearchTree.h"

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
		 * @brief	class for feature matching to use FLANN-tree
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT FLANNtree : public SearchTree
		{
		private:
			CvMat* descriptorStorage;		///< reference descriptor storage
			cv::Mat flannStorage;			///< reference descriptor storage
			cv::flann::Index* flannIndex;	///< openCV tree search interface pointer
			int eMax;						///< limitation of iteration count

		public:
			virtual char* GetFunctionName(){return "FLANNtree";};
			FLANNtree(int eMax=20) : SearchTree()
			{
				/** FLANN-tree support only float type descriptor */
				this->DESCRIPTOR_DATA_TYPE = CV_32F;

				this->descriptorStorage = NULL;
				this->flannIndex = NULL;
				this->eMax = eMax;
			}
			~FLANNtree()
			{
				if(this->descriptorStorage) cvReleaseMat(&this->descriptorStorage);
				if(this->flannIndex) delete this->flannIndex;
			}

			inline void SetEMax(int emax){this->eMax = emax;};
			inline int GetEMax(){return this->eMax;};

			/**
			 * @fn	Training
			 * @brief
			 *		implemantation function to generate descriptor tree
			 * @remark
			 *		the result is member storage to descriptorStorage
			 * @warning
			 *		pointList is not null
			 * @return
			 *		success or failure
			 */
			bool Training(std::vector<windage::FeaturePoint>* pointList);

			/**
			 * @fn	Matching
			 * @brief
			 *		implemantation function to match between input feature point and reference feature points
			 * @remark
			 *		the comparison is each implementation class member storage
			 * @warning
			 *		if difference is NULL pointer that not return the difference value
			 * @return
			 *		matched reference feature index
			 *		reference parameter difference is distance between input feature and matched refernece point
			 */
			int Matching(
						 windage::FeaturePoint point,	///< input feature
						 double* difference = NULL		///< output reference pointer
						 );
		};
		/** @} */ // addtogroup AlgorithmsSearchTree
		/** @} */ // addtogroup Algorithms
	}	
}
#endif // _FLANN_TREE_H_