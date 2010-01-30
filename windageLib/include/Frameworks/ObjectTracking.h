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

#ifndef _OBJECT_TRACKING_H_
#define _OBJECT_TRACKING_H_

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


namespace windage
{
	namespace Frameworks
	{
		class DLLEXPORT ObjectTracking
		{
		protected:
			windage::Calibration* cameraParameter;

			int width;
			int height;
			IplImage* referenceImage;
			std::vector<windage::FeaturePoint> referenceRepository;

			windage::Algorithms::FeatureDetector* detector;
			windage::Algorithms::SearchTree* matcher;
			windage::Algorithms::OpticalFlow* tracker;
			windage::Algorithms::HomographyEstimator* estimator;
			
		public:
			ObjectTracking()
			{
				cameraParameter = NULL;
				referenceImage = NULL;

				detector = NULL;
				matcher = NULL;
				estimator = NULL;
			}
			virtual ~ObjectTracking()
			{
				if(cameraParameter) delete cameraParameter;
				cameraParameter = NULL;
				if(referenceImage) cvReleaseImage(&referenceImage);
				referenceImage = NULL;

				if(detector) delete detector;
				detector = NULL;
				if(matcher) delete matcher;
				matcher = NULL;
				if(estimator) delete estimator;
				estimator = NULL;

				this->referenceRepository.clear();
			}

			inline windage::Calibration* GetCameraParameter(){return this->cameraParameter;};
			inline void SetSize(int width, int height){this->width = width; this->height = height;};
			inline CvSize GetSize(){return cvSize(this->width, this->height);};

			inline void AttatchDetetor(windage::Algorithms::FeatureDetector* detector){this->detector = detector;};
			inline void AttatchMatcher(windage::Algorithms::SearchTree* matcher){this->matcher = matcher;};
			inline void AttatchTracker(windage::Algorithms::OpticalFlow* tracker){this->tracker = tracker;};
			inline void AttatchEstimator(windage::Algorithms::HomographyEstimator* estimator){this->estimator = estimator;};

			bool AttatchReferenceImage(IplImage* grayImage);
			bool Initialize(int width, int height);
			bool UpdateCamerapose(IplImage* grayImage);
		};
	}
}

#endif
