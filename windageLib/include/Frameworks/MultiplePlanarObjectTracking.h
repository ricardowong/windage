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

#ifndef _MULTIPLE_PLANAR_OBJECT_TRACKING_H_
#define _MULTIPLE_PLANAR_OBJECT_TRACKING_H_

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
#include "Algorithms/OpticalFlow.h"
#include "Algorithms/HomographyEstimator.h"
#include "Algorithms/OutlierChecker.h"
#include "Algorithms/HomographyRefiner.h"

#define SearchTreeT windage::Algorithms::FLANNtree

namespace windage
{
	namespace Frameworks
	{
		class DLLEXPORT MultiplePlanarObjectTracking
		{
		protected:
			windage::Calibration* initialCamearParameter;
			std::vector<windage::Calibration*> cameraParameter;
			IplImage* prevImage;

			int width;
			int height;
			double realWidth;
			double realHeight;

			int step;
			int detectionRatio;

			int objectCount;
			std::vector<std::vector<windage::FeaturePoint>> refMatchedKeypoints;
			std::vector<std::vector<windage::FeaturePoint>> sceMatchedKeypoints;

			std::vector<IplImage*> referenceImage;
			std::vector<std::vector<windage::FeaturePoint>> referenceRepository;
			std::vector<SearchTreeT*> searchTree;

			windage::Algorithms::FeatureDetector* detector;
			windage::Algorithms::OpticalFlow* tracker;
			windage::Algorithms::HomographyEstimator* estimator;
			windage::Algorithms::OutlierChecker* checker;
			windage::Algorithms::HomographyRefiner* refiner;

			bool initialize;
			bool trained;
			
		public:
			MultiplePlanarObjectTracking()
			{
				initialCamearParameter = NULL;
				prevImage = NULL;

				objectCount = 0;

				detector = NULL;
				estimator = NULL;
				refiner = NULL;

				initialize = false;
				trained = false;

				step = 0;
				detectionRatio = 0; // automatically allocation depend on reference count
			}
			virtual ~MultiplePlanarObjectTracking()
			{
				if(prevImage) cvReleaseImage(&prevImage);
				prevImage = NULL;

				for(unsigned int i=0; i<this->referenceImage.size(); i++)
					if(referenceImage[i]) cvReleaseImage(&referenceImage[i]);
				this->referenceImage.clear();

				for(unsigned int i=0; i<this->searchTree.size(); i++)
					if(searchTree[i]) delete searchTree[i];
				this->searchTree.clear();
			}

			inline void SetSize(int width, int height){this->width = width; this->height = height;};
			inline CvSize GetSize(){return cvSize(this->width, this->height);};
			inline int GetObjectCount(){return this->objectCount;};

			inline void AttatchCalibration(windage::Calibration* calibration){this->initialCamearParameter = calibration;};
			inline windage::Calibration* GetCameraParameter(int objectID){return this->cameraParameter[objectID];};
			
			inline void AttatchDetetor(windage::Algorithms::FeatureDetector* detector){this->detector = detector;};
			inline void AttatchTracker(windage::Algorithms::OpticalFlow* tracker){this->tracker = tracker;};
			inline void AttatchEstimator(windage::Algorithms::HomographyEstimator* estimator){this->estimator = estimator;};
			inline void AttatchChecker(windage::Algorithms::OutlierChecker* checker){this->checker = checker;};
			inline void AttatchRefiner(windage::Algorithms::HomographyRefiner* refiner){this->refiner = refiner;};

			bool Initialize(int width, int height, double realWidth=640.0, double realHeight=480.0);
			bool AttatchReferenceImage(IplImage* grayImage);
			bool TrainingReference(double scaleFactor=1.0, int scaleStep=1);

			bool UpdateCamerapose(IplImage* grayImage);
			void DrawOutLine(IplImage* colorImage, int objectID, bool drawCross);
			void DrawDebugInfo(IplImage* colorImage, int objectID);
		};
	}
}

#endif
