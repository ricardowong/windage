#ifndef _UTIL_H_
#define _UTIL_H_

#include <cv.h>

#include "wBaekVector.h"
#include "Calibration.h"

class Utils
{
public:
	static void ImmersiveImage(IplImage* image1, IplImage* image2, double rate1=0.5, double rate2=0.5);
	static CvScalar Calc3DPointApproximation(Calibration* lCalibration, Calibration* rCalibration, CvPoint lPoint, CvPoint rPoint);
};

#endif