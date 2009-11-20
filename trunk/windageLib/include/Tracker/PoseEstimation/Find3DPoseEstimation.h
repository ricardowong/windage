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

#ifndef _FIND_3D_POSE_ESTIMATION_H_
#define _FIND_3D_POSE_ESTIMATION_H_

#include <vector>
#include <cv.h>

#include "base.h"
#include "Tracker/Calibration.h"

namespace windage
{
	typedef struct _Matched3DPoint
	{
		CvPoint2D32f pointScene;
		CvPoint3D32f pointReference;
		bool isInlier;

		struct _Matched3DPoint(CvPoint2D32f pointScene, CvPoint3D32f pointReference)
		{
			this->pointScene = pointScene;
			this->pointReference = pointReference;
			isInlier = false;
		}

		void operator=(struct _Matched3DPoint oprd)
		{
			this->pointScene = oprd.pointScene;
			this->pointReference = oprd.pointReference;
			this->isInlier = oprd.isInlier;
		}
	}Matched3DPoint;

	class DLLEXPORT Find3DPoseEstimation
	{
	protected:
		double reprojectionThreshold;
		windage::Calibration* calibration;
		bool useRANSAC;

		std::vector<Matched3DPoint>* matchedPoints;
//		double RunPoseEstimateRANSAC(std::vector<Matched3DPoint>* matchedPoints, double* cameraPose);
	public:
		Find3DPoseEstimation()
		{
			useRANSAC = true;
			reprojectionThreshold = 2.0;
			calibration = NULL;
			matchedPoints = NULL;
		}
		~Find3DPoseEstimation()
		{
		}

		inline void SetUseRANSAC(bool use){this->useRANSAC = use;};
		inline void SetReprojectionThreshold(double reprojectionThreshold=5.0f){this->reprojectionThreshold = reprojectionThreshold;};
		inline void AttatchMatchedPoints(std::vector<Matched3DPoint>* matchedPoints){this->matchedPoints = matchedPoints;};
		inline void AttatchCalibration(windage::Calibration* calibration){this->calibration = calibration;};

		static double ComputeReprojError(CvPoint2D32f scenePoint, CvPoint3D32f refPoint, CvMat* intrinsicMatrix, CvMat* extrinsicMatrix);

		double Calculate();
	};

}

#endif

