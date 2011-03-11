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
 * @file	PlanarObjectTracking.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is framework class for single planar object tracker
 */

#ifndef _PLANAR_OBJECT_TRACKING_H_
#define _PLANAR_OBJECT_TRACKING_H_

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
#include "Algorithms/HomographyRefiner.h"
#include "Algorithms/KalmanFilter.h"

#include "Utilities/Logger.h"

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
		 * @brief	single planar object tracker framework
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT PlanarObjectTracking
		{
		protected:
			static const int MIN_FEATURE_POINTS_COUNT = 9;			///< threshold to determin whether tracked or not

			windage::Calibration* cameraParameter;					///< It is required elements that camera calibration parameter to attatch reference pointer at out-side
			windage::Algorithms::FeatureDetector* detector;			///< It is required elements that feature detection algorithm to attatch reference pointer at out-side
			windage::Algorithms::SearchTree* matcher;				///< It is required elements that feature matching algorithm to attatch reference pointer at out-side
			windage::Algorithms::HomographyEstimator* estimator;	///< It is required elements that homography estimation algorithm to attatch reference pointer at out-side

			windage::Algorithms::OpticalFlow* tracker;				///< It is optional elements that feature tracking algorithm to attatch reference pointer at out-side
			windage::Algorithms::OutlierChecker* checker;			///< It is optional elements that outlier checker algorithm to attatch reference pointer at out-side
			windage::Algorithms::HomographyRefiner* refiner;		///< It is optional elements that homography refinement algorithm to attatch reference pointer at out-side
			windage::Algorithms::KalmanFilter* filter;				///< It is optional elements that kalman filter algorithm to attatch reference pointer at out-side
			int filterStep;

			IplImage* prevImage;									///< gray image for feature tracking
			std::vector<windage::FeaturePoint> refMatchedKeypoints;	///< matched point at reference image
			std::vector<windage::FeaturePoint> sceMatchedKeypoints;	///< matched point at scene image

			int width;												///< input image width
			int height;												///< input image height
			double realWidth;										///< tracking object width
			double realHeight;										///< tracking object height

			int step;												///< current step (detectionRatio < step = detection step)
			int detectionRatio;										///< 1 / detection ratio for tracking step

			IplImage* referenceImage;								///< attatched reference image
			std::vector<windage::FeaturePoint> referenceRepository;	///< reference keypoint repository

			bool initialize;										///< checked initialized
			bool trained;											///< checked trained

			//for performance check
			windage::Logger* logger;
			windage::Logger* performance;
			
		public:
			virtual char* GetFunctionName(){return "PlanarObjectTracking";};
			PlanarObjectTracking()
			{
				prevImage = NULL;
				referenceImage = NULL;

				cameraParameter = NULL;
				detector = NULL;
				matcher = NULL;
				estimator = NULL;
				tracker = NULL;
				checker = NULL;
				refiner = NULL;
				filter = NULL;

				filterStep = 10;

				initialize = false;
				trained = false;

				step = 1;
				detectionRatio = 0;

				logger = NULL;
				performance = NULL;
			}
			virtual ~PlanarObjectTracking()
			{
				if(prevImage) cvReleaseImage(&prevImage);
				prevImage = NULL;
				if(referenceImage) cvReleaseImage(&referenceImage);
				referenceImage = NULL;

				this->referenceRepository.clear();
			}

			inline void SetSize(int width, int height){this->width = width; this->height = height;};
			inline CvSize GetSize(){return cvSize(this->width, this->height);};
			inline void SetDitectionRatio(int ratio){this->detectionRatio=ratio; this->step=ratio+1;};
			inline void SetFilterSetp(int step){this->filterStep = step;};
			inline int GetMatchingCount(){return (int)this->refMatchedKeypoints.size();};
			inline IplImage* GetReferenceImage(){return this->referenceImage;};
			inline void SetLogger(windage::Logger* logger){this->logger = logger;};
			inline void SetPerformance(windage::Logger* logger){this->performance = logger;};
			inline std::vector<windage::FeaturePoint>* GetReferenceRep(){return &this->referenceRepository;};


			inline void ClearTracking(){refMatchedKeypoints.clear();sceMatchedKeypoints.clear();};

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
			 * @fn	AttatchDetetor
			 * @brief
			 *		attatch detection algorithm to member pointer from out-side
			 * @remark
			 *		detection algorithm is to be initialized at out-side
			 * @warning
			 *		It is required elements
			 *		detection algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchDetetor(windage::Algorithms::FeatureDetector* detector){this->detector = detector;};

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
			inline void AttatchEstimator(windage::Algorithms::HomographyEstimator* estimator){this->estimator = estimator;};

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
			 * @fn	AttatchChecker
			 * @brief
			 *		attatch outlier checke algorithm to member pointer from out-side
			 * @remark
			 *		outlier checke algorithm is to be initialized at out-side
			 * @warning
			 *		It is optional elements
			 *		outlier checke algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchChecker(windage::Algorithms::OutlierChecker* checker){this->checker = checker;};

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
			inline void AttatchRefiner(windage::Algorithms::HomographyRefiner* refiner){this->refiner = refiner;};

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
			inline windage::Algorithms::FeatureDetector* GetDetector(){return this->detector;};
			inline windage::Algorithms::SearchTree* GetMatcher(){return this->matcher;};
			inline windage::Algorithms::OpticalFlow* GetTracker(){return this->tracker;};
			inline windage::Algorithms::HomographyEstimator* GetEstimator(){return this->estimator;};
			inline windage::Algorithms::OutlierChecker* GetChecker(){return this->checker;};
			inline windage::Algorithms::HomographyRefiner* GetRefiner(){return this->refiner;};

			/**
			 * @fn	Initialize
			 * @brief
			 *		initialize the parameter and arrange the algorithms
			 * @warning
			 *		It will be called after attatched required elements
			 */
			bool Initialize(int width,					///< input image width
							int height,					///< input image height
							double realWidth=640.0,		///< refenrece object width
							double realHeight=480.0,	///< reference object height
							bool printInfo = true
							);

			/**
			 * @fn	AttatchReferenceImage
			 * @brief
			 *		attatch reference image
			 * @warning
			 *		reference image is always gray image (1-channel)
			 */
			bool AttatchReferenceImage(IplImage* grayImage);

			/**
			 * @fn	TrainingReference
			 * @brief
			 *		training the attatched reference image as multi-scale
			 * @remark
			 *		resize to (1.0 / scaleFactor) * (1 < eachStep <= scaleStep)
			 * @warning
			 *		It will be called after initilization and attatched required elements
			 */
			bool TrainingReference(	double scaleFactor=4.0,	///< resize scale factor
									int scaleStep=8			///< resize scale step
									);

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
			void DrawOutLine(IplImage* colorImage, CvScalar color, bool drawCross);

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

#endif // _PLANAR_OBJECT_TRACKING_H_