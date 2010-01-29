/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo
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
		class DLLEXPORT SearchTree
		{
		protected:
			int DESCRIPTOR_DATA_TYPE;
			double nearestNeighbourhoodRatio;
			
		public:
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

			virtual bool Training(std::vector<windage::FeaturePoint*>* pointList) = 0;
			virtual int Matching(windage::FeaturePoint point, double* difference = NULL) = 0;
		};
	}
}

#endif
