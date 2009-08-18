#include "Tracker.h"
using namespace windage;

Tracker::Tracker()
{
	cameraParameter = NULL;
}

Tracker::~Tracker()
{
	this->Release();
}

void Tracker::Release()
{
	if(cameraParameter) delete cameraParameter;
	cameraParameter = NULL;
}

//void Tracker::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4)
//{
//	this->Release();
//	cameraParameter = new Calibration();
//	cameraParameter->Initialize(fx, fy, cx, cy, d1, d2, d3, d4);
//}

//int Tracker::UpdateCameraPose(IplImage *grayImage)
//{
//	return 0;
//}

void Tracker::DrawInfomation(IplImage *colorImage, double size)
{
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(size, 0.0, 0.0), CV_RGB(255, 0, 0), 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(0.0, size, 0.0), CV_RGB(0, 255, 0), 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(0.0, 0.0, size), CV_RGB(0, 0, 255), 2);
}