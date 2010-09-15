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

#ifndef _MULTIPLE_MARKER_TRACKING_H_
#define _MULTIPLE_MARKER_TRACKING_H_

#include <vector>

#include "Structures/Vector.h"
#include "Structures/Matrix.h"

#include "Structures/Calibration.h"

#include "Algorithms/ChessboardDetector.h"
#include "Algorithms/HomographyRefiner.h"
#include "Algorithms/HomographyEstimator.h"
#include "Algorithms/HomographyRefiner.h"

namespace windage
{
	namespace Frameworks
	{
		class DLLEXPORT MultipleMarkerTracking
		{
		protected:
			int width;
			int height;
			bool initialize;

			windage::Calibration* initialCamearParameter;			///< It is required elements that camera calibration parameter to attatch reference pointer at out-side
			std::vector<windage::Algorithms::ChessboardDetector*> detector;
			windage::Algorithms::HomographyEstimator* estimator;
			windage::Algorithms::HomographyRefiner* refiner;		///< It is optional elements that homography refinement algorithm to attatch reference pointer at out-side

			std::vector<windage::Calibration*> cameraParameter;		///< the number of camera pose is dynamic that is the result camera pose of recodnized and tracked object
			std::vector<windage::Vector3> chessParameter;
		public:
			virtual char* GetFunctionName(){return "MultipleMarkerTracking";};
			MultipleMarkerTracking()
			{
				initialCamearParameter = NULL;

				width = 640;
				height = 480;

				initialize = false;

				estimator = NULL;
				refiner = NULL;
			}
			~MultipleMarkerTracking()
			{
			}

			inline void AttatchCalibration(windage::Calibration* calibration){this->initialCamearParameter = calibration;};
			inline void AttatchEstimator(windage::Algorithms::HomographyEstimator* estimator){this->estimator = estimator;};
			inline void AttatchRefiner(windage::Algorithms::HomographyRefiner* refiner){this->refiner = refiner;};

			inline windage::Calibration* GetCameraParameter(int objectID){return this->cameraParameter[objectID];};
			inline windage::Algorithms::ChessboardDetector* GetDetector(int objectID){return this->detector[objectID];};
			inline windage::Algorithms::HomographyEstimator* GetEstimator(){return this->estimator;};
			inline windage::Algorithms::HomographyRefiner* GetRefiner(){return this->refiner;};
			inline int GetObjectCount(){return (int)this->chessParameter.size();};

			bool Initialize(int width,					///< input image width
							int height,					///< input image height
							bool printInfo = true
							);

			bool AttatchChessboard(int width, int height, double size);
			bool UpdateCamerapose(IplImage* grayImage);
			void DrawOutLine(IplImage* colorImage, int objectID, bool drawCross);
			void DrawDebugInfo(IplImage* colorImage, int objectID);
		};
	}
}


#endif //_MULTIPLE_MARKER_TRACKING_H_