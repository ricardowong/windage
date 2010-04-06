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
 * @file	RotationConverter.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is abstract class to support rotation convertion using static functions
 */

#ifndef _ROTATION_CONVERTER_H_
#define _ROTATION_CONVERTER_H_

#include "Structures/Vector.h"
#include "Structures/Matrix.h"

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
		 * @brief	Abstract class to convert rotation representation
 		 *			refer : http://en.wikipedia.org/wiki/Rotation_representation
		 * @warning QuaternionToEuler function has temporary buf fix code
		 * @author	Woonhyuk Baek
		 */
		class RotationConverter
		{
		private:
			/** can not create object to use only static function */
			RotationConverter()
			{
			};
			~RotationConverter()
			{
			};

		public:
			static windage::Vector4 EulerToQuaternion(windage::Vector3 euler)
			{
				windage::Vector4 quaternion;

				quaternion.x = -cos((euler.x - euler.z)/2.0) * sin(euler.y/2.0);
				quaternion.y =  sin((euler.x - euler.z)/2.0) * sin(euler.y/2.0);
				quaternion.z = -sin((euler.x + euler.z)/2.0) * cos(euler.y/2.0);
				quaternion.w =  cos((euler.x + euler.z)/2.0) * cos(euler.y/2.0);
				return quaternion;
			}

			/**
			 * @warning QuaternionToEuler function has temporary buf fix code
			 */
			static windage::Vector3 QuaternionToEuler(windage::Vector4 quaternion)
			{
				windage::Vector3 euler;

				euler.x =  atan2((quaternion.x*quaternion.z) + (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) - (quaternion.x*quaternion.w));
				euler.y =  acos(-(quaternion.x*quaternion.x) - (quaternion.y*quaternion.y) +(quaternion.z*quaternion.z) + (quaternion.w*quaternion.w));
				euler.z = -atan2((quaternion.x*quaternion.z) - (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) + (quaternion.x*quaternion.w));

				/**
				 * ERROR : z-axis sign error template fix
				 */
//*
				quaternion = RotationConverter::EulerToQuaternion(euler);
				euler.x =  atan2((quaternion.x*quaternion.z) + (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) - (quaternion.x*quaternion.w));
				euler.y =  acos(-(quaternion.x*quaternion.x) - (quaternion.y*quaternion.y) +(quaternion.z*quaternion.z) + (quaternion.w*quaternion.w));
				euler.z = -atan2((quaternion.x*quaternion.z) - (quaternion.y*quaternion.w), (quaternion.y*quaternion.z) + (quaternion.x*quaternion.w));
//*/
				return euler;
			}

			static windage::Matrix3 QuaternionToDcm(windage::Vector4 quaternion)
			{
				windage::Matrix3 dcm;

				dcm.m[0][0] = 1.0 - 2.0*(quaternion.y*quaternion.y + quaternion.z*quaternion.z);
				dcm.m[0][1] = 2.0*		(quaternion.x*quaternion.y - quaternion.z*quaternion.w);
				dcm.m[0][2] = 2.0*		(quaternion.x*quaternion.z + quaternion.y*quaternion.w);

				dcm.m[1][0] = 2.0*		(quaternion.x*quaternion.y + quaternion.z*quaternion.w);
				dcm.m[1][1] = 1.0 - 2.0*(quaternion.x*quaternion.x + quaternion.z*quaternion.z);
				dcm.m[1][2] = 2.0*		(quaternion.y*quaternion.z - quaternion.x*quaternion.w);

				dcm.m[2][0] = 2.0*		(quaternion.x*quaternion.z - quaternion.y*quaternion.w);
				dcm.m[2][1] = 2.0*		(quaternion.x*quaternion.w + quaternion.y*quaternion.z);
				dcm.m[2][2] = 1.0 - 2.0*(quaternion.x*quaternion.x + quaternion.y*quaternion.y);
				return dcm;
			}

			static windage::Vector4 DcmToQuaternion(windage::Matrix3 dcm)
			{
				windage::Vector4 quaternion;

				quaternion.w = sqrt(1.0 + dcm.m[0][0] + dcm.m[1][1] + dcm.m[2][2]) / 2.0;
				quaternion.x = (dcm.m[2][1] - dcm.m[1][2]) / (4.0 * quaternion.w);
				quaternion.y = (dcm.m[0][2] - dcm.m[2][0]) / (4.0 * quaternion.w);
				quaternion.z = (dcm.m[1][0] - dcm.m[0][1]) / (4.0 * quaternion.w);
				return quaternion;
			}

			static windage::Vector3 DcmToEuler(windage::Matrix3 dcm)
			{
				windage::Vector3 eular;
				eular = RotationConverter::QuaternionToEuler(RotationConverter::DcmToQuaternion(dcm));
				return eular;
			}

			static windage::Matrix3 EulerToDcm(windage::Vector3 euler)
			{
				windage::Matrix3 dcm;
				dcm = RotationConverter::QuaternionToDcm(RotationConverter::EulerToQuaternion(euler));
				return dcm;
			}
		};
		/** @} */ // addtogroup Coordinator
	}
}
#endif // _ROTATION_CONVERTER_H_