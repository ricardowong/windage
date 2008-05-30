
#include <vector>
#include <cv.h>

#include "../include/windageMatrix.h"

extern "C" __declspec(dllexport)
void ImageToWorldCoordinate(Vector3* result, Vector3 point, Matrix3 instrinsicMatrix, Matrix3 rotationMatrix, Vector3 translateVector)
{
	Matrix3 baseMatrix, inversMatrix;

	rotationMatrix._13 = rotationMatrix._13 * point.z + translateVector.x;
	rotationMatrix._23 = rotationMatrix._23 * point.z + translateVector.y;
	rotationMatrix._33 = rotationMatrix._33 * point.z + translateVector.z;

	baseMatrix = instrinsicMatrix * rotationMatrix;
	inversMatrix = baseMatrix.Inverse();

	(*result) = inversMatrix * point;

	result->x = result->x / result->z;
	result->y = result->y / result->z;
	result->z = 0;
}

extern "C" __declspec(dllexport)
void WorldToImageCoordinate(Vector3* result, Vector3 point, Matrix3 instrinsicMatrix, Matrix3 rotationMatrix, Vector3 translateVector)
{
	(*result) = instrinsicMatrix * (rotationMatrix * point + translateVector);
}

extern "C" __declspec(dllexport)
void WorldCoordinateCrossPointApproximation(Vector3* result, Vector3* point1, Vector3* point2,
							  double distanceThreshold, Matrix3 k, Matrix3 r1, Matrix3 r2, Vector3 t1, Vector3 t2, Vector3 c1, Vector3 c2)
{
	Vector3 worldPoint1;
	Vector3 worldPoint2;

	Vector3 u, v, d0;
	double s, t;
	double a, b, c, d, e;

	// calculate worldposition
	Matrix3 baseMatrix1 = r1;
	baseMatrix1._13 = t1.x;
	baseMatrix1._23 = t1.y;
	baseMatrix1._33 = t1.z;

	Matrix3 baseProjectionMatrix1 = k * baseMatrix1;
	Matrix3 inverseProjectionMatrix1 = baseProjectionMatrix1.Inverse();

	Matrix3 baseMatrix2 = r2;
	baseMatrix2._13 = t2.x;
	baseMatrix2._23 = t2.y;
	baseMatrix2._33 = t2.z;

	Matrix3 baseProjectionMatrix2 = k * baseMatrix2;
	Matrix3 inverseProjectionMatrix2 = baseProjectionMatrix2.Inverse();

	worldPoint1 = inverseProjectionMatrix1 * (*point1);
	worldPoint1.x /= worldPoint1.z;
	worldPoint1.y /= worldPoint1.z;
	worldPoint1.z = 0;

	worldPoint2 = inverseProjectionMatrix2 * (*point2);
	worldPoint2.x /= worldPoint2.z;
	worldPoint2.y /= worldPoint2.z;
	worldPoint2.z = 0;

	// calculate approximation cross point
	// P0 = worldPoint1, P1 = c1
	u = c1 - worldPoint1;
	// Q0 = worldPoint2, Q1 = c2
	v = c2 - worldPoint2;
	d0 = worldPoint2 - worldPoint1;

	a = u * u;
	b = u * v;
	c = v * v;
	d = u * d0;
	e = v * d0;

	s = (c*d - b*e) / (a*c - b*b);
	t = (b*d - a*e) / (a*c - b*b);

	// result point
	(*result) = ( (worldPoint1 + u*s) + (worldPoint2 + v*t) ) * 0.5f;
}

extern "C" __declspec(dllexport)
void FindWorldCoordinatePointApproximation(std::vector<Vector3>* result, std::vector<Vector3>* pointList1, std::vector<Vector3>* pointList2,
							  double distanceThreshold, Matrix3 k, Matrix3 r1, Matrix3 r2, Vector3 t1, Vector3 t2, Vector3 c1, Vector3 c2,
							  IplImage* debugImage)
{
	result->clear();
	int count1 = (int)pointList1->size();
	int count2 = (int)pointList2->size();

	if(count1 == 0 || count2 == 0)
	{
		return;
	}

	if(distanceThreshold < 5)
		distanceThreshold = 5;

	Vector3 point1;
	Vector3 point2;
	Vector3 resultPoint;

	Vector3 worldPoint1;
	Vector3 worldPoint2;

	Vector3 projectionPoint;
	Vector3 projectionCameraPoint;

	Vector3 u, v, d0;
	double s, t;
	double a, b, c, d, e;

	double x1, x2, x3, y1, y2, y3;
	double la, lb;

	double distance;
	double min;
	int minIndex;

	Vector3 temp;

	Matrix3 baseMatrix1 = r1;
	baseMatrix1._13 = t1.x;
	baseMatrix1._23 = t1.y;
	baseMatrix1._33 = t1.z;

	Matrix3 baseProjectionMatrix1 = k * baseMatrix1;
	Matrix3 inverseProjectionMatrix1 = baseProjectionMatrix1.Inverse();

	Matrix3 baseMatrix2 = r2;
	baseMatrix2._13 = t2.x;
	baseMatrix2._23 = t2.y;
	baseMatrix2._33 = t2.z;

	Matrix3 baseProjectionMatrix2 = k * baseMatrix2;
	Matrix3 inverseProjectionMatrix2 = baseProjectionMatrix2.Inverse();

	projectionCameraPoint = k * (r2 * c1 + t2);
	projectionCameraPoint.x /= projectionCameraPoint.z;
	projectionCameraPoint.y /= projectionCameraPoint.z;
	projectionCameraPoint.z = 1;

	x1 = projectionCameraPoint.x;
	y1 = projectionCameraPoint.y;

	for(int i=0; i<count1; i++)
	{
		point1 = (*pointList1)[i];

		worldPoint1 = inverseProjectionMatrix1 * point1;
		worldPoint1.x /= worldPoint1.z;
		worldPoint1.y /= worldPoint1.z;
		worldPoint1.z = 0;

		projectionPoint = k * (r2 * worldPoint1 + t2);
		projectionPoint.x /= projectionPoint.z;
		projectionPoint.y /= projectionPoint.z;
		projectionPoint.z = 1;

		x2 = projectionPoint.x;
		y2 = projectionPoint.y;
		
		minIndex = 0;
		min = 99999;
		for(int j=0; j<count2; j++)
		{
			point2 = (*pointList2)[j];

			x3 = point2.x;
			y3 = point2.y;

			la = y1 - y2;
			lb = x2 - x1;

			distance = fabs( x1*y2 + x2*y3 + x3*y1 - x3*y2 - x2*y1 - x1*y3 ) / sqrt( pow(la, 2) + pow(lb, 2) );
			if(distance < min)
			{
				min = distance;
				minIndex = j;
			}

		}

		if(min < distanceThreshold)
		{
			point2 = (*pointList2)[minIndex];

			worldPoint2 = inverseProjectionMatrix2 * point2;
			worldPoint2.x /= worldPoint2.z;
			worldPoint2.y /= worldPoint2.z;
			worldPoint2.z = 0;

			// P0 = worldPoint1, P1 = c1
			u = c1 - worldPoint1;
			// Q0 = worldPoint2, Q1 = c2
			v = c2 - worldPoint2;
			d0 = worldPoint2 - worldPoint1;

			a = u * u;
			b = u * v;
			c = v * v;
			d = u * d0;
			e = v * d0;

			s = (c*d - b*e) / (a*c - b*b);
			t = (b*d - a*e) / (a*c - b*b);

			if( -0.5f < s && s < 1.0f &&
				-0.5f < t && t < 1.0f )
			{
				// result point
				resultPoint = ( (worldPoint1 + u*s) + (worldPoint2 + v*t) ) * 0.5f;
				result->push_back(resultPoint);
			}

		}

		// draw epipolar line
		if(debugImage != NULL)
		{
			cvLine(debugImage, cvPoint((int)x1, (int)y1), cvPoint((int)x2, (int)y2), CV_RGB(255, 0, 0));
		}

	}

}