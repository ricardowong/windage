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
 * @file	Logger.h
 * @author	Woonhyuk Baek
 * @version 1.0
 * @date	2010.03.01
 * @brief	It is log class for logging at reconstruction informations
 */

#ifndef _LOADER_H_
#define _LOADER_H_

#include <vector>
#include <string>

#include <cv.h>

#include "base.h"
#include "Structures/Calibration.h"
#include "Structures/ReconstructionPoint.h"
#include "Utilities/Logger.h"

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
		 * @brief	Class for Logging at processtime and programmer's message
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT Loader
		{
		private:
			std::vector<windage::Calibration*>* calibrationList;
			std::vector<std::string>* filenameList;
			std::vector<windage::ReconstructionPoint>* reconstructionPoints;

		public:
			Loader()
			{
				calibrationList = NULL;
				filenameList = NULL;
				reconstructionPoints = NULL;
			}
			~Loader()
			{
			}

			inline void AttatchCalibration(std::vector<windage::Calibration*>* calibration){this->calibrationList = calibration;};
			inline void AttatchFilename(std::vector<std::string>* filename){this->filenameList = filename;};
			inline void AttatchReconstructionPoints(std::vector<windage::ReconstructionPoint>* reconstructionPoints){this->reconstructionPoints = reconstructionPoints;};
			
			bool DoLoad(const char* filename="");
		};
		/** @} */ // addtogroup Reconstruction
	}
}

#endif