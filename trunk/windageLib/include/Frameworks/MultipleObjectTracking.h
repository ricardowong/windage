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
 * @file	MultipleObjectTracking.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is framework class for multiple planar object tracker
 */

#ifndef _MULTIPLE_OBJECT_TRACKING_H_
#define _MULTIPLE_OBJECT_TRACKING_H_

#include <vector>
#include <map>

#include <cv.h>
#include "base.h"

#include "Structures/Vector.h"
#include "Structures/Matrix.h"

#include "Structures/FeaturePoint.h"
#include "Structures/Calibration.h"

#include "Algorithms/FeatureDetector.h"
#include "Algorithms/SearchTree.h"
#include "Algorithms/FLANNtree.h"
#include "Algorithms/KDtree.h"
#include "Algorithms/OpticalFlow.h"
#include "Algorithms/PoseEstimator.h"
#include "Algorithms/EPnPRANSACestimator.h"
#include "Algorithms/OpenCVRANSACestimator.h"

#include "Algorithms/PoseRefiner.h"

/** pre-selected search tree algorithm whenever can change other search tree algorithm  */
#define SearchTreeT windage::Algorithms::FLANNtree
#define SEARCH_TREE_RATIO 0.5
#define PoseEstimationT windage::Algorithms::OpenCVRANSACestimator

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
		 * @brief	multiple planar object tracker framework
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT MultipleObjectTracking
		{
		protected:
			static const int MIN_FEATURE_POINTS_COUNT = 10;			///< threshold to determin whether tracked or not

			windage::Calibration* initialCamearParameter;			///< It is required elements that camera calibration parameter to attatch reference pointer at out-side
			windage::Algorithms::FeatureDetector* detector;			///< It is required elements that feature detection algorithm to attatch reference pointer at out-side
			windage::Algorithms::OpticalFlow* tracker;				///< It is required elements that feature tracking algorithm to attatch reference pointer at out-side
			
			windage::Algorithms::PoseEstimator* estimator;			///< It is required elements that homography estimation algorithm to attatch reference pointer at out-side
			windage::Algorithms::PoseRefiner* refiner;				///< It is optional elements that homography refinement algorithm to attatch reference pointer at out-side
			std::vector<PoseEstimationT*> estimatorList;	///< the number of matching algorithm is dynamic that is training of the reference image

			std::vector<windage::Calibration*> cameraParameter;		///< the number of camera pose is dynamic that is the result camera pose of recodnized and tracked object
			std::vector<SearchTreeT*> searchTree;					///< the number of matching algorithm is dynamic that is training of the reference image

			IplImage* prevImage;									///< gray image for feature tracking
			
			int objectCount;

			int width;												///< input image width
			int height;												///< input image height
			double realWidth;										///< tracking object width
			double realHeight;										///< tracking object height

			int step;												///< current step (step == objectID = detection step)
			int detectionRatio;										///< always detection and tracking step for multiple object
			int detectionStep;
			
			bool initialize;										///< checked initialized
			bool trained;											///< checked trained

		public:
			std::vector<std::vector<windage::FeaturePoint>> refMatchedKeypoints;	///< matched point at reference image
			std::vector<std::vector<windage::FeaturePoint>> sceMatchedKeypoints;	///< matched point at scene image
			std::vector<std::vector<windage::FeaturePoint>> referenceRepository;	///< reference keypoint repository

			bool update;
			bool processThread;
			int objectID;

		public:
			virtual char* GetFunctionName(){return "MultipleObjectTracking";};
			MultipleObjectTracking()
			{
				initialCamearParameter = NULL;
				prevImage = NULL;

				objectCount = 0;

				detector = NULL;
				estimator = NULL;
				refiner = NULL;

				initialize = false;
				trained = false;

				update = false;
				processThread = true;
				objectID = 0;

				step = 0;
				/** automatically allocation depend on reference count */
				detectionRatio = 2;
				detectionStep = 0;

			}
			virtual ~MultipleObjectTracking()
			{
				if(prevImage) cvReleaseImage(&prevImage);
				prevImage = NULL;

				for(unsigned int i=0; i<this->estimatorList.size(); i++)
					if(estimatorList[i]) delete estimatorList[i];
				this->estimatorList.clear();

				for(unsigned int i=0; i<this->searchTree.size(); i++)
					if(searchTree[i]) delete searchTree[i];
				this->searchTree.clear();
			}

			inline void SetSize(int width, int height){this->width = width; this->height = height;};
			inline CvSize GetSize(){return cvSize(this->width, this->height);};
			inline void SetDitectionRatio(int ratio){if(ratio<1) ratio=1; this->detectionRatio=ratio; this->step=ratio+1;};
			inline int GetObjectCount(){return this->objectCount;};
			inline int GetMatchingCount(int i){return (int)this->refMatchedKeypoints[i].size();};

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
			inline void AttatchCalibration(windage::Calibration* calibration){this->initialCamearParameter = calibration;};
			
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
			 * @fn	AttatchTracker
			 * @brief
			 *		attatch tracker algorithm to member pointer from out-side
			 * @remark
			 *		tracker algorithm is to be initialized at out-side
			 * @warning
			 *		It is required elements
			 *		tracker algorithm is not create in-side at this class so do not release this pointer
			 */
			inline void AttatchTracker(windage::Algorithms::OpticalFlow* tracker){this->tracker = tracker;};
			
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

			inline windage::Calibration* GetCameraParameter(int objectID){return this->cameraParameter[objectID];};
			inline windage::Algorithms::FeatureDetector* GetDetector(){return this->detector;};
			inline windage::Algorithms::SearchTree* GetMatcher(int objectID){return this->searchTree[objectID];};
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
			 * @fn	DrawDebugInfo
			 * @brief
			 *		draw matched feature points
			 */
			void DrawDebugInfo(IplImage* colorImage, int objectID);
		};
		/** @} */ // addtogroup Frameworks
	}
}
#endif // _MULTIPLE_OBJECT_TRACKING_H_