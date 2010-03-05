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
 * @file	SIFTGPUdetector.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.03.04
 * @brief	It is SIFT feature detection & description class using GPU
 */

#ifndef _SIFT_GPU_DETECTOR_H_
#define _SIFT_GPU_DETECTOR_H_

#include <vector>

#include <cv.h>
#include <SiftGPU.h>

#include "base.h"
#include "Structures/Vector.h"
#include "Algorithms/FeatureDetector.h"

namespace windage
{
	namespace Algorithms
	{
		/**
		 * @defgroup Algorithms Algorithm classes
		 * @brief
		 *		algorithm classes
		 * @addtogroup Algorithms
		 * @{
		 */

		/**
		 * @defgroup AlgorithmsFeatureDetector Feature Detector
		 * @brief
				feature detector algorithm classes
		 * @addtogroup AlgorithmsFeatureDetector
		 * @{
		 */

		/**
		 * @brief	Class for SIFT GPU feature detector
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT SIFTGPUdetector : public FeatureDetector
		{
		private:
			SiftGPU* sift;

		public:
			SIFTGPUdetector(int numberOfPyramid = 1) : FeatureDetector()
			{
				sift = NULL;
				sift = CreateNewSiftGPU(numberOfPyramid);
				sift->SetVerbose(0);
				sift->VerifyContextGL();
				sift->CreateContextGL();
			}
			~SIFTGPUdetector()
			{
				if(sift != NULL) delete sift;
			}

			/**
			 * @fn	DoExtractKeypointsDescriptor
			 * @brief
			 *		implemantation of SIFT feature extraction & description
			 * @remark
			 *		the result is depend on threshold (member valuable)
			 * @warning
			 *		input image is always gray image (1-channel)
			 * @return
			 *		success or failure
			 */
			bool DoExtractKeypointsDescriptor(IplImage* grayImage);
		};
		/** @} */ // addtogroup AlgorithmsFeatureDetector
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _SIFT_DETECTOR_H_