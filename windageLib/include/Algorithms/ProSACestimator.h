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
		class DLLEXPORT ProSACestimator : public HomographyEstimator
		{
		private:
			int maxIteration;
		public:
			ProSACestimator() : HomographyEstimator()
			{
				this->reprojectionError = 2.0;
				this->maxIteration = 1000;
			}
			~ProSACestimator()
			{
			}

			inline void SetMaxIteration(int iteration){this->maxIteration = iteration;};
			bool Calculate();
		};

		// for stl sort
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

			bool operator==(class MatchedPoint  oprd)
			{
				return this->distance == oprd.distance;
			}
			bool operator<(class MatchedPoint  oprd)
			{
				return this->distance < oprd.distance;
			}
		};

		typedef struct _CompareDistanceLess
		{
			bool operator()(const windage::Algorithms::MatchedPoint& p, const windage::Algorithms::MatchedPoint& q) const
			{
				return p.distance < q.distance;
			}
		}CompareDistanceLess;
	}
}

#endif
