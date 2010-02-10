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
 * @file	StereoReconstruction.h
 * @author	Woonhyuk Baek
 * @version 1.0
 * @date	2010.02.04
 * @brief	reconstruction class using matched feature at two view images
 */

#ifndef _STEREO_RECONSTRUCTION_H_
#define _STEREO_RECONSTRUCTION_H_

#include <vector>

#include <cv.h>

#include "base.h"

#include "Structures/Vector.h"
#include "Structures/Matrix.h"
#include "Structures/Calibration.h"
#include "Structures/FeaturePoint.h"
#include "Structures/ReconstructionPoint.h"

namespace windage
{
	namespace Reconstruction
	{
		/**
		 * @defgroup Reconstruction Reconstruction classes
		 * @brief
		 *		Reconstruction classes
		 * @addtogroup Reconstruction
		 * @{
		 */

		/**
		 * @brief	reconstruction class using matched feature at two view images
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT StereoReconstruction
		{
		private:
			int maxIteration;										///< RANSAC max iteration count
			double confidence;
			double reprojectionError;								///< threshold to determin outlier or not

			windage::Calibration* initialCameraParameter;			///< base camera parameter to attatch reference pointer at out-side
			windage::Calibration* localCameraParameter;				///< updating camera parameter to attatch reference pointer at out-side
			std::vector<windage::FeaturePoint>* matchedPoint1;		///< matched point list at first camera image to attatch reference pointer at out-side
			std::vector<windage::FeaturePoint>* matchedPoint2;		///< matched point list at second camera image to attatch reference pointer at out-side

			std::vector<windage::Vector3> normalizedMatchedPoint1;	///< normalized matched point list
			std::vector<windage::Vector3> normalizedMatchedPoint2;	///< normalized matched point list

			std::vector<windage::ReconstructionPoint> reconstructionPoints;		///< reconstructed point list

			CvMat *essentialMatrix;									/// temporary essential matrix
			int inlierCount;

		public:
			StereoReconstruction(void)
			{
				this->reprojectionError = 2.0;
				this->confidence = 0.995;
				this->maxIteration = 2000;
				essentialMatrix = cvCreateMat(3, 3, CV_64F);
				inlierCount = 0;
			}
			~StereoReconstruction(void)
			{
				if(essentialMatrix) cvReleaseMat(&essentialMatrix);
				essentialMatrix = NULL;
			}

			inline std::vector<windage::ReconstructionPoint>* GetReconstructionPoints(){return &this->reconstructionPoints;};
			inline void SetReprojectionError(double error){this->reprojectionError = error;};
			inline double GetReprojectionError(){return this->reprojectionError;};
			inline void SetConfidence(int confidence){this->confidence = confidence;};
			inline int GetInlierCount(){return this->inlierCount;};

			inline void AttatchBaseCameraParameter(windage::Calibration* cameraParameter){this->initialCameraParameter = cameraParameter;};
			inline void AttatchUpdateCameraParameter(windage::Calibration* cameraParameter){this->localCameraParameter = cameraParameter;};
			inline windage::Calibration* GetBaseCameraParameter(){return this->initialCameraParameter;};
			inline windage::Calibration* GetUpdateCameraParameter(){return this->localCameraParameter;};
			
			inline void AttatchMatchedPoint1(std::vector<windage::FeaturePoint>* matchedPoint1){this->matchedPoint1 = matchedPoint1;};
			inline void AttatchMatchedPoint2(std::vector<windage::FeaturePoint>* matchedPoint2){this->matchedPoint2 = matchedPoint2;};

			/**
			 * @fn	CalculateNormalizedPoint
			 * @brief
			 *		convert matched point list to normalized matched point list
			 * @warning
			 *		It is required elements
			 */
			void CalculateNormalizedPoint();

			bool CalibratedTriangulation(CvMat *matR, CvMat *matT, CvMat *ptL, CvMat *ptR, CvMat *pt3D);
			bool DecomposeEMatrix(CvMat *EMat);
			int ReconstructAll(CvMat *matE);
			int CountInliers(double thresh, double *err);

			bool ComputeEssentialMatrix8Points(CvMat *pt1, CvMat *pt2, CvMat *EMat);

			/**
			 * @fn	ComputeEssentialMatrixRANSAC
			 * @brief
			 *		compute essential matrix and reconstruction 3D points
			 */
			bool ComputeEssentialMatrixRANSAC(double* error);
		};
		/** @} */ // addtogroup Reconstruction
	}

}
#endif // _STEREO_RECONSTRUCTION_H_