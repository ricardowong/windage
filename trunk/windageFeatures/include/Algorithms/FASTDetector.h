/* ========================================================================
 * PROJECT: windage Features
 * ========================================================================
 * This work is based on the original windage Features developed by
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

#ifndef _FAST_DETECTOR_
#define _FAST_DETECTOR_

#include <vector>
#include <cv.h>

#include <base.h>
#include "Structures/FeaturePoint.h"
#include "Algorithms/FeatureExtractor.h"

namespace windage
{
	namespace Algorithms
	{
		class FASTDetector : public FeatureExtractor
		{
		protected:
			int FASTindex;

		public:
			FASTDetector()
			{
				this->FASTindex = 9;
				this->threshold = 30.0;
				this->NAME = "FASTDetector";
			}
			~FASTDetector()
			{
			}
			
			void SetIndex(int index){this->FASTindex = index;}
			bool DoExtractFeature(IplImage* grayImage);
		};
	}
}

#endif //_FAST_DETECTOR_