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

#ifndef _CHESSBOARD_TRACKER_H_
#define _CHESSBOARD_TRACKER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Tracker.h"

namespace windage
{
	/**
	 * @brief
	 *		Class for Camera Tracker using chessboard
	 * @author
	 *		windage
	 */
	class DLLEXPORT ChessboardTracker:public Tracker
	{
	private:
		CvPoint2D32f* chessboardPoints;	///< storage for chessboard position at image coordinate

		int chessboardWidth;			///< chessboard block count
		int chessboardHeight;			///< chessboard block count
		double fieldSize;				///< chessboard block size

		bool usingSubpixelAccuracy;

		void Release();

		inline void SetChessboard(int width, int height, double size){this->chessboardWidth=width, this->chessboardHeight=height; this->fieldSize=size;};
		inline int GetPointCount(){return (this->chessboardWidth-1) * (this->chessboardHeight-1);};

		int FindChessBoardCorner(IplImage* inputImage);
		bool UpdateExtrinsicParams();

	public:
		ChessboardTracker();
		virtual ~ChessboardTracker();

		inline void SetUsingSubpixelAccuracy(bool use){this->usingSubpixelAccuracy = use;};

		/**
		 * @brief
		 *		Initialize Chessboard Tracker
		 * @remark
		 *		initialize chessboard tracker setting the chessboard information and camera paramter (intrinsic parameter)
		 */
		void Initialize(double fx, 				///< intrinsic parameter x focal length
						double fy, 				///< intrinsic parameter y focal length
						double cx, 				///< intrinsic parameter x principle point
						double cy, 				///< intrinsic parameter y principle point
						double d1, 				///< intrinsic parameter distortion factor1
						double d2, 				///< intrinsic parameter distortion factor2
						double d3, 				///< intrinsic parameter distortion factor3
						double d4, 				///< intrinsic parameter distortion factor4
						int chessboardWidth = 7,	///< chessboard block width count
						int chessboardHeight = 8,	///< chessboard block height count
						double fieldSize = 28.0		///< chessboard block size
						);

		/**
		 * @brief
		 *		Update Camera Pose (extrinsic parameter)
		 * @remark
		 *		update extrinsic parameter method using input image
		 */
		int UpdateCameraPose(IplImage* grayImage);

		/**
		 * @brief
		 *		Draw Debug Information
		 * @remark
		 *		draw debug information method
		 */
		void DrawDebugInfo(IplImage* colorImage);
	};
}

#endif