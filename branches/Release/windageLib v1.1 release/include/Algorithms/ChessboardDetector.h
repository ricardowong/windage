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
 * @file	ChessboardDetector.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is chessboard detection class
 */

#ifndef _CHESSBOARD_DETECTOR_H_
#define _CHESSBOARD_DETECTOR_H_

#include <vector>

#include <cv.h>
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
		 * @brief	Class for Chessboard detector
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT ChessboardDetector
		{
		private:
			std::vector<windage::FeaturePoint> keypoints;	///< repository for coner poitns
			std::vector<windage::FeaturePoint> reference;
			int widthCount;
			int heightCount;
			double cellSize;

		public:
			virtual char* GetFunctionName(){return "ChessboardDetector";};
			ChessboardDetector(int widthCount=8, int heightCount=7, double cellSize = 28.0)
			{
				SetChessboard(widthCount, heightCount, cellSize);
			}
			~ChessboardDetector()
			{
			}

			inline std::vector<windage::FeaturePoint>* GetKeypoints(){return &this->keypoints;};
			inline std::vector<windage::FeaturePoint>* GetReferencePoints(){return &this->reference;}
			void SetChessboard(int widthCount=8, int heightCount=7, double cellSize = 28.0)
			{
				this->widthCount = widthCount;
				this->heightCount = heightCount;
				this->cellSize = cellSize;

				int size = (widthCount-1) * (heightCount-1);
				reference.resize(size);
				keypoints.resize(size);

				int index = 0;
				for(int y=heightCount-2; y>=0; y--)
				{
					for(int x=0; x<widthCount-1; x++)
					{
						reference[index].SetPoint(windage::Vector3(x*cellSize, y*cellSize, 1.0));
						index++;
					}
				}
			}
			
			void DrawMarkerInfo(IplImage* image);
			bool FindMarker(IplImage* grayImage);
		};
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _CHESSBOARD_DETECTOR_H_