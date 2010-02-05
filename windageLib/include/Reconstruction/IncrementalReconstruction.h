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
 * @file	IncrementalReconstruction.h
 * @author	Woonhyuk Baek
 * @version 1.0
 * @date	2010.02.04
 * @brief	reconstruction class using matched feature at multi view images
 */

#ifndef _INCREMENTAL_RECONSTRUCTION_H_
#define _INCREMENTAL_RECONSTRUCTION_H_

#include <vector>

#include <cv.h>

#include "base.h"

#include "Structures/Vector.h"
#include "Structures/Matrix.h"
#include "Structures/Calibration.h"
#include "Structures/FeaturePoint.h"
#include "Structures/ReconstructionPoint.h"

#include "Algorithms/SearchTree.h"

#include "Reconstruction/StereoReconstruction.h"

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
		 * @brief	reconstruction class using matched feature at multi view images
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT IncrementalReconstruction
		{
		private:
			int sceneCount;
			windage::Calibration* initialCameraParameter;
			std::vector<windage::Calibration*> cameraParameters;

			windage::Algorithms::SearchTree* searchtree;

			std::vector<std::vector<windage::ReconstructionPoint>> reconstructionPoints;
			std::vector<std::vector<windage::FeaturePoint>> featurePointsList;
		public:
			IncrementalReconstruction()
			{
				sceneCount = 0;
				initialCameraParameter = NULL;
				searchtree = NULL;
			}
			~IncrementalReconstruction()
			{
				for(unsigned int i=0; i<cameraParameters.size(); i++)
					delete cameraParameters[i];
				cameraParameters.clear();
			}

			inline void AttatchCalibration(windage::Calibration* calibration){this->initialCameraParameter = calibration;};
			inline void AttatchSearchTree(windage::Algorithms::SearchTree* matcher){this->searchtree = matcher;};
			inline windage::Calibration* GetCameraParameter(int i){return cameraParameters[i];};
			inline std::vector<windage::ReconstructionPoint>* GetReconstructedPoint(int i){return &(reconstructionPoints[i]);};
			
			void AttatchFeaturePoint(std::vector<windage::FeaturePoint>* featurePoints);
			bool Calculate();
		};
	}
}

#endif // _INCREMENTAL_RECONSTRUCTION_H_