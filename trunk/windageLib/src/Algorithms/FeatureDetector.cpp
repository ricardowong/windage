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

#include "Algorithms/FeatureDetector.h"
using namespace windage;
using namespace windage::Algorithms;

void FeatureDetector::DrawKeypoint(IplImage* colorImage, FeaturePoint point, CvScalar color)
{
	windage::Vector3 keypointPT = point.GetPoint();
	double size = (double)(point.GetSize()/2);
	double dir = point.GetDir();
	double halfPI = CV_PI / 2.0;

	double x = size;
	double y = size;

	CvPoint pointList[4];
	for(int i=0; i<4; i++)
	{
		double sinDir = sin(-dir + halfPI*i);
		double cosDir = cos(-dir + halfPI*i);

		int dx = cvRound(keypointPT.x + (cosDir * x - sinDir * y));
		int dy = cvRound(keypointPT.y + (sinDir * x + cosDir * y));

		pointList[i] = cvPoint(dx, dy);
	}
	
	for(int i=0; i<4; i++)
	{
		int i2 = i==3?0:i+1;
		cvLine(colorImage, pointList[i], pointList[i2], color, 2);
	}

	double sinDir = sin((-dir+45) + halfPI);
	double cosDir = cos((-dir+45) + halfPI);
	int dx = cvRound(keypointPT.x + (cosDir * x - sinDir * y) * 1.5);
	int dy = cvRound(keypointPT.y + (sinDir * x + cosDir * y) * 1.5);
	cvLine(colorImage, cvPoint(keypointPT.x, keypointPT.y), cvPoint(dx, dy), CV_RGB(color.val[0], color.val[1], color.val[2]), 2);
}

void FeatureDetector::DrawKeypoints(IplImage* colorImage, CvScalar color)
{
	for(unsigned int i=0; i<keypoints.size(); i++)
	{
		DrawKeypoint(colorImage, keypoints[i], color);
	}
}