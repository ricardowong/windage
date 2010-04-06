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
 * @file	ARForOpenGL.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is class to support AR applicaations using OPENGL(with GLUT)
 */

#ifndef _AUGMENTED_REALITY_FOR_OPENGL_H_
#define _AUGMENTED_REALITY_FOR_OPENGL_H_

#include "base.h"

#include <gl/glut.h>
#include "AugmentedReality.h"

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
		 * @brief	Class for Augmented Reality Tool at OpenGL Environment
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT ARForOpenGL : public AugmentedReality
		{
		private:
			GLuint backgroundTexture;	///< OpenGL Texture Pointer
			void Release();

		public:
			ARForOpenGL() : AugmentedReality()
			{
				backgroundTexture = 0;
			};
			~ARForOpenGL()
			{
				this->Release();
			};

			/**
			 * @fn	Initialize
			 * @brief
			 *		Initialize ARTool for OpenGL
			 * @remark
			 *		initialize ARTool for OpenGL Environment
			 */
			void Initialize(
							int imageWidth,			///< input background image width size
							int imageHeight,		///< input background image height size
							bool isFlip=false,		///< input flip background and modelview matrix
							int textureWidth=512	///< input texture image size (width == height) (*necessary multiple of 4)
							);

			/**
			 * @fn	DrawBackgroundTexture
			 * @brief
			 *		Draw Background Texture
			 * @remark
			 *		draw background image abstract method using input image
			 */
			void DrawBackgroundTexture(IplImage* inputImage);

			/**
			 * @fn	SetProjectionMatrix
			 * @brief
			 *		Set Projection Matrix
			 * @remark
			 *		set projection matrix abstract method using intrinsic matrix
			 */
			void SetProjectionMatrix();

			/**
			* @fn	SetModelViewMatrix
			 * @brief
			 *		Set ModelView Matrix
			 * @remark
			 *		set model-view matrix abstract method using extrinsic matrix
			 */
			void SetModelViewMatrix();
		};
		/** @} */ // * @addtogroup Coordinator
	}
}
#endif // _AUGMENTED_REALITY_FOR_OPENGL_H_