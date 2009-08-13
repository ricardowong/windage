#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_

#include <cv.h>

class LaserSegmentation
{
private:
	bool initialized;
	IplImage* segmentedImage;

	void Release();

public:
	LaserSegmentation();
	~LaserSegmentation();

	bool Initialize(IplImage* tempImage);
	IplImage* Segment(IplImage* inputImage, double threshold=220);
	inline IplImage* GetSegmentedImage(){return segmentedImage;};

	IplImage* FillHole();
	CvPoint GetMaxContour(int sizeThreshold = 5);
};

#endif