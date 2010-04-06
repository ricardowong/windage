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
 * @file	SingleObjectTracking.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is framework class for single 3d object tracker
 */

#ifndef _SINGLE_OBJECT_TRACKING_H_
#define _SINGLE_OBJECT_TRACKING_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/Vector.h"
#include "Structures/Matrix.h"

#include "Structures/FeaturePoint.h"

#include "Structures/Calibration.h"

#include "Algorithms/FeatureDetector.h"
#include "Algorithms/SearchTree.h"
#include "Algorithms/OpticalFlow.h"
#include "Algorithms/HomographyEstimator.h"
#include "Algorithms/OutlierChecker.h"
#include "Algorithms/PoseRefiner.h"
#include "Algorithms/KalmanFilter.h"

namespace windage
{
	namespace Frameworks
	{
		/**
		 * @defgroup Frameworks Framework classes
		 * @brief
		 *		framework classes
		 * @addtogroup Frameworks
		 * @{
		 */

		/**
		 * @brief	single 3D object tracker framework
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT SingleObjectTracking
		{
		protected:
			static const int MIN_FEATURE_POINTS_COUNT = 9;			///< threshold to determin whether tracked or not

			windage::Calibration* cameraParameter;					///< It is required elements that camera calibration parameter to attatch reference pointer at out-side
			windage::Algorithms::SearchTree* matcher;				///< It is required elements that feature matching algorithm to attatch reference pointer at out-side
			windage::Algorithms::PoseEstimator* estimator;			///< It is required elements that homography estimation algorithm to attatch reference pointer at out-side

			windage::Algorithms::OpticalFlow* tracker;				///< It is optional elements that feature tracking algorithm to attatch reference pointer at out-side
			windage::Algorithms::PoseRefiner* refiner;				///< It is optional elements that homography refinement algorithm to attatch reference pointer at out-side
			windage::Algorithms::KalmanFilter* filter;				///< It is optional elements that kalman filter algorithm to attatch reference pointer at out-side
			int filterStep;

			IplImage* prevImage;									///< gray image for feature tracking
			
			int width;												///< input image width
			int height;												///< input image height

			int step;												///< current step (detectionRatio < step = detection step)
			int detectionRatio;										///< 1 / detection ratio for tracking step

			bool initialize;										///< checked initialized
			bool trained;											///< checked trained

		public:
			std::vector<windage::FeaturePoint> referenceRepository;	///< reference keypoint repository
			std::vector<windage::FeaturePoint> refMatchedKeypoints;	///< matched point at reference image
			std::vector<windage::FeaturePoint> sceMatchedKeypoints;	///< matched point at scene image

			bool update;
			bool processThread;
			
		public:
			virtual char* GetFunctionName(){return "SingleObjectTracking";};
			SingleObjectTracking()
			{
				prevImage = NULL;

				cameraParameter = NULL;
				matcher = NULL;
				estimator = NULL;
				tracker = NULL;
				refiner = NULL;
				filter = NULL;

				filterStep = 10;

				initialize = false;
				trained = false;

				update = false;
				processThread = true;

				step = 1;
				detectionRatio = 0;
			}
			virtual ~SingleObjectTracking()
			{
				if(prevImage) cvReleaseImage(&prevImage);
				prevImage = NULL;

				this->referenceRepository.clear();
			}

			inline void SetSize(int width, int height){this->width = width; this->height = height;};
			inline CvSize GetSize(){return cvSize(this->width, this->height);};
			inline void SetDitectionRatio(int ratio){this->detectionRatio=ratio; this->step=ratio+1;};
			inline void SetFilterSetp(int step){this->filterStep = step;};
			inline int GetMatchingCount(){return (int)this->refMatchedKeypoints.size();};

			/**
			 * @fn	AttatchCalibration
			 * @brief
			 *		attatch camera parameter to member pointer from out-side
			 * @remark
			 *		camera parameter is to be initialized at out-side
			 * @warning
			 *		It is required elements
			 *		camera parameter is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchCalibration(windage::Calibration* calibration){this->cameraParameter = calibration;};
			
			/**
			 * @fn	AttatchMatcher
			 * @brief
			 *		attatch matching algorithm to member pointer from out-side
			 * @remark
			 *		matching algorithm is to be initialized at out-side
			 * @warning
			 *		It is required elements
			 *		matching algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchMatcher(windage::Algorithms::SearchTree* matcher){this->matcher = matcher;};

			/**
			 * @fn	AttatchEstimator
			 * @brief
			 *		attatch homography estimation algorithm to member pointer from out-side
			 * @remark
			 *		estimation algorithm is to be initialized at out-side
			 * @warning
			 *		It is required elements
			 *		homography estimation algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchEstimator(windage::Algorithms::PoseEstimator* estimator){this->estimator = estimator;};

			/**
			 * @fn	AttatchTracker
			 * @brief
			 *		attatch tracker algorithm to member pointer from out-side
			 * @remark
			 *		tracker algorithm is to be initialized at out-side
			 * @warning
			 *		It is optional elements
			 *		tracker algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchTracker(windage::Algorithms::OpticalFlow* tracker){this->tracker = tracker;};

			/**
			 * @fn	AttatchRefiner
			 * @brief
			 *		attatch homography refinement algorithm to member pointer from out-side
			 * @remark
			 *		homography refinement algorithm is to be initialized at out-side
			 * @warning
			 *		It is optional elements
			 *		homography refinement algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchRefiner(windage::Algorithms::PoseRefiner* refiner){this->refiner = refiner;};

			/**
			 * @fn	AttatchRefiner
			 * @brief
			 *		attatch kalman filter algorithm to member pointer from out-side
			 * @remark
			 *		kalman filter algorithm is to be initialized at out-side
			 * @warning
			 *		It is optional elements
			 *		kalman filter algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchFilter(windage::Algorithms::KalmanFilter* filter){this->filter = filter;};

			inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};
			inline windage::Algorithms::SearchTree* GetMatcher(){return this->matcher;};
			inline windage::Algorithms::OpticalFlow* GetTracker(){return this->tracker;};
			inline windage::Algorithms::PoseEstimator* GetEstimator(){return this->estimator;};
			inline windage::Algorithms::PoseRefiner* GetRefiner(){return this->refiner;};

			/**
			 * @fn	Initialize
			 * @brief
			 *		initialize the parameter and arrange the algorithms
			 * @warning
			 *		It will be called after attatched required elements
			 */
			bool Initialize(int width,					///< input image width
							int height,					///< input image height
							bool printInfo = true
							);

			/**
			 * @fn	TrainingReference
			 * @brief
			 *		training the attatched reference image as multi-scale
			 * @remark
			 *		resize to (1.0 / scaleFactor) * (1 < eachStep <= scaleStep)
			 * @warning
			 *		It will be called after initilization and attatched required elements
			 */
			bool TrainingReference(std::vector<windage::FeaturePoint>* referenceFeatures);

			/**
			 * @fn	UpdateCamerapose
			 * @brief
			 *		calculate camera pose using input image
			 * @warning
			 *		input image is always gray image (1-channel)
			 *		It will be called after initilization and training
			 */
			bool UpdateCamerapose(IplImage* grayImage);

			/**
			 * @fn	DrawOutLine
			 * @brief
			 *		draw recognized or tracked object outlier
			 */
			void DrawOutLine(IplImage* colorImage, bool drawCross);

			/**
			 * @fn	DrawDebugInfo
			 * @brief
			 *		draw matched feature points
			 */
			void DrawDebugInfo(IplImage* colorImage);
		};
		/** @} */ // addtogroup Frameworks
	}
}
#endif // _SINGLE_OBJECT_TRACKING_H_