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

double Reconstructor::CalculatePlaneError(Vector4 plane, Vector3 point)
{
	Vector3 normal = Vector3(plane.x, plane.y, plane.z);
	double error = abs(normal * point + plane.w) / normal.getLength();
	return error;
}

Vector4 Reconstructor::PlaneEstimation(Vector3 point1, Vector3 point2, Vector3 point3)
{
	Vector3 direction1 = point2 - point1;
	Vector3 direction2 = point3 - point1;
	
	Vector3 plane = direction1^direction2;
	double d = - (plane * point1);

	return Vector4(plane.x, plane.y, plane.z, d);
}

#include <time.h>
Vector4 Reconstructor::PlaneEstimationRANSAC(std::vector<Vector3>* points, Vector3& center, std::vector<Vector3>* consensusPoints)
{
	srand(time(NULL));
	Vector4 bestResult;

	Vector3 bestSum;
	double bestCount = 0;
	double bestError = 9999999;
	Vector4 bestPlane;

	for(int i=0; i<points->size() * 100; i++)
	{
		int index[3];
		
		index[0] = rand()%points->size();
		index[1] = rand()%points->size();
		index[2] = rand()%points->size();

		while(index[0] == index[1] || index[0] == index[2] || index[1] == index[2])
		{
			index[0] = rand()%points->size();
			index[1] = rand()%points->size();
			index[2] = rand()%points->size();
		}

		Vector3 point1 = (*points)[index[0]];
		Vector3 point2 = (*points)[index[1]];
		Vector3 point3 = (*points)[index[2]];

		Vector4 plane = PlaneEstimation(point1, point2, point3);

		const double ERROR_THRESHOLD = 1.0;

		Vector3 sum = Vector3();
		int count = 0;
		double error = 0.0;
		for(int j=0; j<points->size(); j++)
		{
			double tempError = CalculatePlaneError(plane, (*points)[j]);
			if(tempError < ERROR_THRESHOLD)
			{
				error += tempError;
				count++;
				sum += (*points)[j];
			}
		}
		error /= count;
		sum /= count;

		if(count >= bestCount)
		{
			if(count == bestCount)
			{
				if(error < bestError)
				{
					bestPlane = plane;
					bestCount = count;
					bestError = error;
					bestSum = sum;

					if(consensusPoints)
					{
						consensusPoints->clear();
						for(int j=0; j<points->size(); j++)
						{
							double tempError = CalculatePlaneError(plane, (*points)[j]);
							if(tempError < ERROR_THRESHOLD)
							{
								consensusPoints->push_back((*points)[j]);
							}
						}
					}
				}
			}
			else
			{
				bestPlane = plane;
				bestCount = count;
				bestError = error;
				bestSum = sum;

				if(consensusPoints)
				{
					consensusPoints->clear();
					for(int j=0; j<points->size(); j++)
					{
						double tempError = CalculatePlaneError(plane, (*points)[j]);
						if(tempError < ERROR_THRESHOLD)
						{
							consensusPoints->push_back((*points)[j]);
						}
					}
				}
			}			
		}
	}

	center = bestSum;
	return bestPlane;
}