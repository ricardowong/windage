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

#ifndef _3D_RECONSTRUCTOR_H_
#define _3D_RECONSTRUCTOR_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Tracker/Calibration.h"

namespace windage
{
	/**
	 * @brief
	 *		Class for 3D Reconstruction
	 * @author
	 *		windage
	 */
	class DLLEXPORT Reconstructor
	{
	public:
		/**
		 * @brief
		 *		Stereo-based 3D Reconstruction Method
		 * @remark
		 *		Calculate 3D Point from Approximate projection line at stereo-camera
		 */
		static CvScalar Calc3DPointApproximation(
													Calibration* lCalibration,	///< stereo-camera1 parameter
													Calibration* rCalibration,	///< stereo-camera2 parameter
													CvPoint lPoint,				///< stereo-camera1 point based on Image coordinate
													CvPoint rPoint				///< stereo-camera2 point based on Image coordinate
												);
	};
}

#endif