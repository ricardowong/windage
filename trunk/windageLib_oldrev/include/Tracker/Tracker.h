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

#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <cv.h>

#include "base.h"
#include "Calibration.h"

namespace windage
{
	enum POSE_ESTIMATION_METHOD{RANSAC = 0, LMEDS, PROSAC, POSE_3D};

	/**
	 * @brief
	 *		Abstract Class for Camera Tracker
	 * @author
	 *		windage
	 */
	class DLLEXPORT Tracker
	{
	protected:
		Calibration* cameraParameter;	///< camera parameter
		virtual void Release();
		void DecomposeHomographyToRT(CvMat *intrinsic, CvMat *Homography, CvMat *RT);
		
	public:
		Tracker();
		virtual ~Tracker();

//		void Initialize(double fx, double fy, double cx, double cy, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
		inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};
		
		/**
		 * @brief
		 *		Update Camera Pose (extrinsic parameter)
		 * @remark
		 *		update extrinsic parameter abstract method using input image
		 */
		virtual double UpdateCameraPose(IplImage* grayImage) = 0;

		/**
		 * @brief
		 *		Draw Debug Information
		 * @remark
		 *		draw debug information abstract method
		 */
		virtual void DrawDebugInfo(IplImage* colorImage) = 0;

		/**
		 * @brief
		 *		Draw 3-Axis
		 * @remark
		 *		draw axis line for confirming tracked object information
		 */
		void DrawInfomation(IplImage* colorImage, double size = 10.0);

		void Initialize(double fx, 				///< intrinsic parameter x focal length
						double fy, 				///< intrinsic parameter y focal length
						double cx, 				///< intrinsic parameter x principle point
						double cy, 				///< intrinsic parameter y principle point
						double d1, 				///< intrinsic parameter distortion factor1
						double d2, 				///< intrinsic parameter distortion factor2
						double d3, 				///< intrinsic parameter distortion factor3
						double d4 				///< intrinsic parameter distortion factor4
						);
	};

}
#endif