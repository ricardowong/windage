#ifndef _UTILS_H_
#define _UTILS_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Tracker/Calibration.h"

namespace windage
{
	class DLLEXPORT Utils
	{
	public:
		static void DrawTextToImage(IplImage* colorImage, CvPoint position, char* message);
		static void DrawWorldCoordinatePoint(IplImage* colorImage, Calibration* calibration, CvScalar worldPoint, double size=1.0, bool drawText=false);
	};
}

#endif