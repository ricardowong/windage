#ifndef _TRACKER_H_
#define _TRACKER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Calibration.h"

namespace windage
{

	class DLLEXPORT Tracker
	{
	protected:
		Calibration* cameraParameter;
		virtual void Release();
		
	public:
		Tracker();
		virtual ~Tracker();

//		void Initialize(double fx, double fy, double cx, double cy, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
		inline Calibration* GetCameraParameter(){return this->cameraParameter;};
		
		virtual int UpdateCameraPose(IplImage* grayImage) = 0;
		virtual void DrawDebugInfo(IplImage* colorImage) = 0;
		void DrawInfomation(IplImage* colorImage, double size = 10.0);
	};

}
#endif