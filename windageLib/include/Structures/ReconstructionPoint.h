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
 * @file	ReconstructionPoint.h
 * @author	Woonhyuk Baek
 * @version 1.0
 * @date	2010.02.05
 * @brief	It has reconstruction point information
 */

#ifndef _RECONSTRUCTION_POINT_H_
#define _RECONSTRUCTION_POINT_H_

#include <cv.h>

#include "base.h"
#include "Vector.h"
#include "FeaturePoint.h"

namespace windage
{
	/**
	 * @defgroup Structures Data Structures
	 * @brief
	 *		data structures classes
	 * @addtogroup Structures
	 * @{
	 */

	/**
	 * @brief	Class for reconstruction points
	 * @author	Woonhyuk Baek
	 */
	class DLLEXPORT ReconstructionPoint
	{
	protected:
		windage::Vector4 point;		///< position of reconstruction point (w-value is 1.0)
		std::vector<windage::FeaturePoint> featureList;
		CvScalar color;				///< color of reconstruction point
		int objectID;				///< object id (initialize -1)
		bool outlier;				///< checked outlier

	public:
		ReconstructionPoint()
		{
			point = windage::Vector4(0.0, 0.0, 0.0, 1.0);
			this->featureList.clear();
			color = CV_RGB(255, 255, 255);
			objectID = -1;
			outlier = false;
		}
		~ReconstructionPoint()
		{
		}

		/**
		 * @fn	operator=
		 * @brief
		 *		overwriting assignment expression
		 * @remark
		 *		copy all data
		 */
		virtual void operator=(ReconstructionPoint oprd)
		{
			this->point = oprd.GetPoint();

			std::vector<windage::FeaturePoint>* featureList = oprd.GetFeatureList();
			this->featureList.clear();
			for(unsigned int i=0; i<featureList->size(); i++)
				this->featureList.push_back((*featureList)[i]);

			this->color = oprd.GetColor();
			this->objectID = oprd.GetObjectID();
			this->outlier = oprd.IsOutlier();
		}

		inline void SetPoint(windage::Vector4 point){this->point = point;};
		inline windage::Vector4 GetPoint(){return this->point;};
		inline void AddFeaturePoint(windage::FeaturePoint feature){this->featureList.push_back(feature);};
		inline std::vector<windage::FeaturePoint>* GetFeatureList(){return &this->featureList;};
		inline windage::FeaturePoint GetFeature(int i){return this->featureList[i];};

		inline void SetColor(CvScalar color=CV_RGB(255, 255, 2555)){this->color = color;};
		inline CvScalar GetColor(){return this->color;};
		inline void SetObjectID(int id){this->objectID = id;};
		inline int GetObjectID(){return this->objectID;};
		inline void SetOutlier(bool outlier){this->outlier = outlier;};
		inline bool IsOutlier(){return this->outlier;};
	};
	/** @} */ // addtogroup Structures
}

#endif // _FEATURE_POINT_H_