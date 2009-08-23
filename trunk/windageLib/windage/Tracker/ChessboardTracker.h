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
	class DLLEXPORT ChessboardTracker:public Tracker
	{
	private:
		CvPoint2D32f* chessboardPoints;

		int chessboardWidth;
		int chessboardHeight;
		double fieldSize;

		void Release();

		inline void SetChessboard(int width, int height, double size){this->chessboardWidth=width, this->chessboardHeight=height; this->fieldSize=size;};
		inline int GetPointCount(){return (this->chessboardWidth-1) * (this->chessboardHeight-1);};

		int FindChessBoardCorner(IplImage* inputImage);
		bool UpdateExtrinsicParams();

	public:
		ChessboardTracker();
		virtual ~ChessboardTracker();

		void Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, int chessboardWidth, int chessboardHeight, double fieldSize);

		int UpdateCameraPose(IplImage* grayImage);
		void DrawDebugInfo(IplImage* colorImage);
	};
}

#endif