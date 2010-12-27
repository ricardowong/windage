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

#ifndef _FEATURE_POINT_H_
#define _FEATURE_POINT_H_

#include <vector>

#include <cv.h>
#include "base.h"
#include "Vector.h"

namespace windage
{
	class DLLEXPORT FeaturePoint
	{
	protected:
		windage::Vector3 point;	///< position of feature point (z-value is 1.0)
		double size;				///< feature size
		double dir;				///< feature orientation

	public:
		FeaturePoint()
		{
			size = 0;
			dir = 0;
		}
		virtual ~FeaturePoint()
		{
		}

		virtual void operator=(FeaturePoint oprd)
		{
			this->point = oprd.GetPoint();
			this->size = oprd.GetSize();
			this->dir = oprd.GetDir();
		}

		inline void SetPoint(windage::Vector3 point){this->point = point;};
		inline windage::Vector3 GetPoint(){return this->point;};
		inline void SetSize(double size){this->size = size;};
		inline double GetSize(){return this->size;};
		inline void SetDir(double dir){this->dir = dir;};
		inline double GetDir(){return this->dir;};
	};
}

#endif // _FEATURE_POINT_H_