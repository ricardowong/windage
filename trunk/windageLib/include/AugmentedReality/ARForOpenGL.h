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

#ifndef _AUGMENTED_REALITY_FOR_OPENGL_H_
#define _AUGMENTED_REALITY_FOR_OPENGL_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <gl/glut.h>
#include "AugmentedReality.h"

namespace windage
{
	/**
	 * @brief
	 *		Class for Augmented Reality Tool at OpenGL Environment
	 * @author
	 *		windage
	 */
	class DLLEXPORT ARForOpenGL : public AugmentedReality
	{
	private:
		GLuint backgroundTexture;	///< OpenGL Texture Pointer

		void Release();

	public:
		ARForOpenGL();
		~ARForOpenGL();

		/**
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

		void DrawBackgroundTexture(IplImage* inputImage);
		void SetProjectionMatrix();
		void SetModelViewMatrix();
	};
}

#endif