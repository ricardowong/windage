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

#include "Reconstructor.h"
#include "Utils/wMatrix.h"
using namespace windage;

CvScalar Reconstructor::Calc3DPointApproximation(Calibration* lCalibration, Calibration* rCalibration, CvPoint lPoint, CvPoint rPoint)
{
	CvPoint2D64f temp1, temp2;

	temp1 = lCalibration->ConvertImage2World(lPoint.x, lPoint.y, 0.0);
	temp2 = lCalibration->ConvertImage2World(lPoint.x, lPoint.y, 1000.0);
	Vector3 worldPoint1 = Vector3(temp1.x, temp1.y, 0.0);
	Vector3 u = Vector3(temp2.x - temp1.x, temp2.y - temp1.y, 1000.0 - 0.0);

	temp1 = rCalibration->ConvertImage2World(rPoint.x, rPoint.y, 0.0);
	temp2 = rCalibration->ConvertImage2World(rPoint.x, rPoint.y, 1000.0);
	Vector3 worldPoint2 = Vector3(temp1.x, temp1.y, 0.0);
	Vector3 v = Vector3(temp2.x - temp1.x, temp2.y - temp1.y, 1000.0 - 0.0);
	
	Vector3 d0 = worldPoint2 - worldPoint1;	

	Vector3 temp;
	double a = u * u;
	double b = u * v;
	double c = v * v;
	double d = u * d0;
	double e = v * d0;

	double s = (c*d - b*e) / (a*c - b*b);
	double t = (b*d - a*e) / (a*c - b*b);

	Vector3 result = ( (worldPoint1 + u*s) + (worldPoint2 + v*t) ) * 0.5f;

	return cvScalar(result.x, result.y, result.z);
}