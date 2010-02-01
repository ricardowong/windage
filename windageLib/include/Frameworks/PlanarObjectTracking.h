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

namespace windage
{
	namespace Frameworks
	{
		class DLLEXPORT PlanarObjectTracking
		{
		protected:
			static const int MIN_FEATURE_POINTS_COUNT = 10;

			windage::Calibration* cameraParameter;
			IplImage* prevImage;
			std::vector<windage::FeaturePoint> refMatchedKeypoints;
			std::vector<windage::FeaturePoint> sceMatchedKeypoints;

			int width;
			int height;
			double realWidth;
			double realHeight;

			int step;
			int detectionRatio;

			IplImage* referenceImage;
			std::vector<windage::FeaturePoint> referenceRepository;

			windage::Algorithms::FeatureDetector* detector;
			windage::Algorithms::SearchTree* matcher;
			windage::Algorithms::OpticalFlow* tracker;
			windage::Algorithms::HomographyEstimator* estimator;
			windage::Algorithms::OutlierChecker* checker;
			windage::Algorithms::HomographyRefiner* refiner;

			bool initialize;
			bool trained;
			
		public:
			PlanarObjectTracking()
			{
				prevImage = NULL;
				referenceImage = NULL;

				cameraParameter = NULL;
				detector = NULL;
				matcher = NULL;
				estimator = NULL;
				refiner = NULL;

				initialize = false;
				trained = false;

				step = 1;
				detectionRatio = 0;
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
			inline int GetMatchingCount(){return (int)this->refMatchedKeypoints.size();};

			inline void AttatchCalibration(windage::Calibration* calibration){this->cameraParameter = calibration;};
			inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};
			
			inline void AttatchDetetor(windage::Algorithms::FeatureDetector* detector){this->detector = detector;};
			inline void AttatchMatcher(windage::Algorithms::SearchTree* matcher){this->matcher = matcher;};
			inline void AttatchTracker(windage::Algorithms::OpticalFlow* tracker){this->tracker = tracker;};
			inline void AttatchEstimator(windage::Algorithms::HomographyEstimator* estimator){this->estimator = estimator;};
			inline void AttatchChecker(windage::Algorithms::OutlierChecker* checker){this->checker = checker;};
			inline void AttatchRefiner(windage::Algorithms::HomographyRefiner* refiner){this->refiner = refiner;};

			bool Initialize(int width, int height, double realWidth=640.0, double realHeight=480.0);
			bool AttatchReferenceImage(IplImage* grayImage);
			bool TrainingReference(double scaleFactor=1.0, int scaleStep=1);

			bool UpdateCamerapose(IplImage* grayImage);
			void DrawOutLine(IplImage* colorImage, bool drawCross);
			void DrawDebugInfo(IplImage* colorImage);
		};
	}
}

#endif
