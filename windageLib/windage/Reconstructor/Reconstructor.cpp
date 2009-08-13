#include "Utils.h"

void Utils::ImmersiveImage(IplImage* image1, IplImage* image2, double rate1, double rate2)
{
	#pragma omp parallel for
	for(int y=0; y<image1->height; y++)
	{
		for(int x=0; x<image1->width; x++)
		{
			CvScalar color = cvGet2D(image1, y, x);
			CvScalar color2 = cvGet2D(image2, y, x);
			double gray = color2.val[0] + color2.val[1] + color2.val[2];
			if(gray > 0)
			{
				color.val[0] = color.val[0] * rate1 + color2.val[0] * rate2;
				color.val[1] = color.val[1] * rate1 + color2.val[1] * rate2;
				color.val[2] = color.val[2] * rate1 + color2.val[2] * rate2;
			}

			cvSet2D(image1, y, x, color);
		}
	}
}


CvScalar Utils::Calc3DPointApproximation(Calibration* lCalibration, Calibration* rCalibration, CvPoint lPoint, CvPoint rPoint)
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