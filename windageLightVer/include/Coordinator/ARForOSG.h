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
 * @file	ARForOSG.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is class to support AR applicaations using OSG
 */

#ifndef _AUGMENTED_REALITY_FOR_OSG_H_
#define _AUGMENTED_REALITY_FOR_OSG_H_

#include "base.h"

#include "AugmentedReality.h"
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
		 * @brief	Class for Augmented Reality Tool at OSG Environment (hidding)
		 * @warning	It need to convert windage matrix to OSG matrix for AR rednering
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT ARForOSG : public AugmentedReality
		{
		private:
			int width;							///< rendering image width
			int height;							///< rendering image height

			void Release();
		public:
			ARForOSG()
			{
				this->width = 640;
				this->height = 480;
				cameraParameter = NULL;
			}
			~ARForOSG()
			{
				this->Release();
			}

			/**
			 * @fn	Initialize
			 * @brief
			 *		Initialize ARTool for OSG
			 * @remark
			 *		initialize ARTool for OSG Environment
			 */
			void Initialize(
							int width,	///< rendering image width
							int height	///< rendering image height
							);

			/**
			 * @fn	DrawBackgroundTexture
			 * @brief
			 *		Draw Background Texture
			 * @warning
			 *		do not work
			 * @remark
			 *		do not work
			 */
			void DrawBackgroundTexture(IplImage* inputImage);

			/**
			 * @fn	SetProjectionMatrix
			 * @brief
			 *		update projection matrix convert from intrsic matrix
			 * @remark
			 *		update to projectionMatrix member value
			 */
			void SetProjectionMatrix();

			/**
			 * @fn	SetModelViewMatrix
			 * @brief
			 *		update model-view matrix convert from extrinsic matrix
			 * @remark
			 *		update to modelviewMatrix member value
			 */
			void SetModelViewMatrix();

			/**
			 * @fn	ConvertModelViewMatrix
			 * @brief
			 *		static function to convert from extrinsic matrix to OSG model-view matrix
			 * @return
			 *		OSG model-view matrix
			 */
			static windage::Matrix4 ConvertModelViewMatrix(windage::Matrix4 extrinsic);


			static windage::Vector3 ConvertOSGPosition(windage::Vector3 position);
		};
		/** @} */ // * @addtogroup Coordinator
	}
}
#endif // _AUGMENTED_REALITY_FOR_OSG_H_