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
 * @file	FeatureExportor.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.03.01
 * @brief	It is log class for logging at reconstruction informations
 */

#ifndef _FEATURE_POINT_EXPORTOR_H_
#define _FEATURE_POINT_EXPORTOR_H_

#include <vector>
#include <string>

#include <cv.h>

#include "base.h"
#include "Structures/Calibration.h"
#include "Structures/ReconstructionPoint.h"
#include "Utilities/Logger.h"

namespace windage
{
	/**
	 * @brief	Class for Logging at feature data
	 * @author	Woonhyuk Baek
	 */
	class DLLEXPORT FeatureExportor
	{
	private:
		windage::Logger* logger;
		std::string function_name;
		std::vector<windage::FeaturePoint>* featurePoints;

	public:
		FeatureExportor()
		{
			logger = NULL;
			featurePoints = NULL;
			function_name = std::string("");
		}
		~FeatureExportor()
		{
		}

		inline void SetFunctionName(std::string function_name){this->function_name = function_name;};
		inline void AttatchLogger(windage::Logger* logger){this->logger = logger;};
		inline void SetFeaturePoints(std::vector<windage::FeaturePoint>* featurePoints){this->featurePoints = featurePoints;};
		
		bool DoExport();
	};
}

#endif //_FEATURE_POINT_EXPORTOR_H_
