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
 * @file	MultiMarkerCoordinator.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class to calcuate relation of multi-markers
 */

#ifndef _MULTI_MARKER_COORDINATE_H_
#define _MULTI_MARKER_COORDINATE_H_

#include <cv.h>

#include "base.h"
#include "Structures/Matrix.h"
#include "Structures/Calibration.h"

namespace windage
{
	namespace Coordinator
	{
		/**
		 * @defgroup Coordinator Coordinator
		 * @brief
		 *		coordinator classes
		 * @addtogroup Coordinator
		 * @{
		 */

		/**
		 * @brief	Class for multi-marker coordination
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT MultiMarkerCoordinator
		{
		public:
			/**
			 * @fn	GetTranslation
			 * @brief
			 *		static function to calculate translation from baseCalibration to toCalibration
			 * @return
			 *		translation vector
			 */
			static Vector3 GetTranslation(Calibration* baseCalibration, Calibration* toCalibration);

			/**
			 * @fn	GetRotation
			 * @brief
			 *		static function to calculate rotation from baseCalibration to toCalibration
			 * @return
			 *		rotation matrix
			 */
			static Matrix3 GetRotation(Calibration* baseCalibration, Calibration* toCalibration);

			/**
			 * @fn	CalculateExtrinsic
			 * @brief
			 *		static function to calculate extrinsic matrix base on baseCalibration using traslation and rotation
			 * @return
			 *		extrinsic matrix
			 */
			static Matrix4 CalculateExtrinsic(Calibration* baseCalibration, Matrix3 toRotation, Vector3 toTranslation);

			static Matrix4 GetRelation(Calibration* baseCalibration, Calibration* toCalibration);
			static Matrix4 CalculateExtrinsic(Calibration* baseCalibration, Matrix4 relation);
		};
		/** @} */ // addtogroup Coordinator
	}
}
#endif // _MULTI_MARKER_COORDINATE_H_