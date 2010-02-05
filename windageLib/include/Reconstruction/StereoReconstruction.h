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

#ifndef _STEREO_RECONSTRUCTION_H_
#define _STEREO_RECONSTRUCTION_H_

#include <vector>

#include <cv.h>

#include "base.h"

#include "Structures/Vector.h"
#include "Structures/Matrix.h"
#include "Structures/Calibration.h"
#include "Structures/FeaturePoint.h"

namespace windage
{
	namespace Reconstruction
	{
		class DLLEXPORT StereoReconstruction
		{
		private:
			double reprojectionError;

			windage::Calibration* initialCameraParameter;
			windage::Calibration* localCameraParameter;
			std::vector<windage::FeaturePoint>* matchedPoint1;
			std::vector<windage::FeaturePoint>* matchedPoint2;

			std::vector<windage::Vector3> normalizedMatchedPoint1;
			std::vector<windage::Vector3> normalizedMatchedPoint2;

			std::vector<bool> isInlierList;
			std::vector<windage::Vector4> reconstructionPoints;

			CvMat *essentialMatrix;

		public:
			StereoReconstruction(void)
			{
				reprojectionError = 2.0;
				essentialMatrix = cvCreateMat(3, 3, CV_64F);
			}
			~StereoReconstruction(void)
			{
				if(essentialMatrix) cvReleaseMat(&essentialMatrix);
				essentialMatrix = NULL;
			}

			inline std::vector<bool>* GetIsInlierChecker(){return &this->isInlierList;};
			inline std::vector<windage::Vector4>* GetReconstructionPoints(){return &this->reconstructionPoints;};
			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};

			inline void AttatchBaseCameraParameter(windage::Calibration* cameraParameter){this->initialCameraParameter = cameraParameter;};
			inline void AttatchUpdateCameraParameter(windage::Calibration* cameraParameter){this->localCameraParameter = cameraParameter;};
			inline windage::Calibration* GetBaseCameraParameter(){return this->initialCameraParameter;};
			inline windage::Calibration* GetUpdateCameraParameter(){return this->localCameraParameter;};
			
			inline void AttatchMatchedPoint1(std::vector<windage::FeaturePoint>* matchedPoint1){this->matchedPoint1 = matchedPoint1;};
			inline void AttatchMatchedPoint2(std::vector<windage::FeaturePoint>* matchedPoint2){this->matchedPoint2 = matchedPoint2;};

			void CalculateNormalizedPoint();

			bool CalibratedTriangulation(CvMat *matR, CvMat *matT, CvMat *ptL, CvMat *ptR, CvMat *pt3D);
			bool DecomposeEMatrix(CvMat *EMat);
			int ReconstructAll(CvMat *matE);
			int CountInliers(double thresh, double *err);

			bool ComputeEssentialMatrix8Points(CvMat *pt1, CvMat *pt2, CvMat *EMat);
			bool ComputeEssentialMatrixRANSAC(double* error);			
		};
	}

}
#endif // _STEREO_RECONSTRUCTION_H_