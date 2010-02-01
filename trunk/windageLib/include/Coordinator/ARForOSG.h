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
		 * @brief
		 *		Class for Augmented Reality Tool at OSG Environment (hidding)
		 * @author
		 *		windage
		 */
		class DLLEXPORT ARForOSG : public AugmentedReality
		{
		private:
			void Release();
			int width;
			int height;
			windage::Matrix4 projectionMatrix;
			windage::Matrix4 modelviewMatrix;

		public:
			ARForOSG();
			~ARForOSG();

			/**
			 * @brief
			 *		Initialize ARTool for OpenGL
			 * @remark
			 *		initialize ARTool for OpenGL Environment
			 */
			void Initialize(int width, int height);

			void SetProjectionMatrix();
			void SetModelViewMatrix();
			void DrawBackgroundTexture(IplImage* inputImage);

			inline windage::Matrix4 GetProjectionMatrix(){return this->projectionMatrix;};
			inline windage::Matrix4 GetModelViewMatrix(){return this->modelviewMatrix;};
			static windage::Matrix4 CalculateModelViewMatrix(windage::Matrix4 extrinsic);
		};
	}
}


#endif
