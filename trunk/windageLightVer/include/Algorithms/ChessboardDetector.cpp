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

#include "Algorithms/ChessboardDetector.h"

using namespace windage;
using namespace windage::Algorithms;

void ChessboardDetector::DrawMarkerInfo(IplImage* image)
{
	double r = 255.0;
	double g = 0.0;
	double b = 0.0;

	cvCircle(image, cvPoint((int)keypoints[0].GetPoint().x, (int)keypoints[0].GetPoint().y), 5, CV_RGB(255, 0, 0), CV_FILLED);
	int pointCount = (int)keypoints.size();
	for(int i=1; i<pointCount; i++)
	{
		r = 255.0 - 255.0 * (i/(double)pointCount);
		b = 255.0 * (i/(double)pointCount);

		cvLine(image, cvPoint((int)keypoints[i-1].GetPoint().x, (int)keypoints[i-1].GetPoint().y), cvPoint((int)keypoints[i].GetPoint().x, (int)keypoints[i].GetPoint().y), CV_RGB(r, g, b), 2);
		cvCircle(image, cvPoint((int)keypoints[i].GetPoint().x, (int)keypoints[i].GetPoint().y), 5, CV_RGB(r, g, b), CV_FILLED);
	}
}

bool ChessboardDetector::FindMarker(IplImage* grayImage)
{
	if(this->widthCount <= 0 || this->heightCount <= 0 || this->cellSize <= 0)
		return false;

	CvSize size = cvSize(this->widthCount-1, this->heightCount-1);
	int pointCount = size.width * size.height;
	CvPoint2D32f* chessboardPoints = new CvPoint2D32f[pointCount];

	int result = cvFindChessBoardCornerGuesses(grayImage, NULL, NULL, size, chessboardPoints, &pointCount);
	if(result == 0)
		return false;

	cvFindCornerSubPix(grayImage, chessboardPoints, pointCount, cvSize(5,5), cvSize(-1,-1),
		cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

	for(int i=0; i<pointCount; i++)
	{
		keypoints[i].SetPoint(windage::Vector3(chessboardPoints[i].x, chessboardPoints[i].y, 1.0));
	}

	if(chessboardPoints) delete chessboardPoints;

	return true;
}