#ifndef _3D_RECONSTRUCTOR_H_
#define _3D_RECONSTRUCTOR_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Tracker/Calibration.h"

namespace windage
{
	class DLLEXPORT Reconstructor
	{
	public:
		static CvScalar Calc3DPointApproximation(Calibration* lCalibration, Calibration* rCalibration, CvPoint lPoint, CvPoint rPoint);
	};
}

#endif