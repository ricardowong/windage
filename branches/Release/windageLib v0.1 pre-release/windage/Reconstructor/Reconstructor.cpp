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