#ifndef _UTILS_H_
#define _UTILS_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>

namespace windage
{
	class DLLEXPORT Utils
	{
	public:
		static void DrawTextToImage(IplImage* colorImage, CvPoint position, char* message);
	};
}

#endif