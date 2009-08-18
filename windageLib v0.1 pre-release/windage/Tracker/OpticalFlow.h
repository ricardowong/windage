#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>

namespace windage
{
	class OpticalFlow
	{
	private:
		int imageWidth;
		int imageHeight;

		CvSize windowSize;
		int pyramidLevel;
		
		const static int maxPointCount = 1000;
		CvPoint2D32f feature1[maxPointCount];
		CvPoint2D32f feature2[maxPointCount];
		char foundFeature[maxPointCount];
		float errorFeature[maxPointCount];

		CvTermCriteria terminationCriteria;
		IplImage* prevImage;
		IplImage* pyramid1;
		IplImage* pyramid2;

		void Release();
	public:
		OpticalFlow();
		~OpticalFlow();

		inline void SetImageSize(int width, int height){this->imageWidth=width;this->imageHeight=height;};
		inline CvSize GetImageSize(){return cvSize(this->imageWidth, this->imageHeight);};
		inline void SetWindowSize(CvSize size=cvSize(10, 10)){this->windowSize = size;};
		inline CvSize GetWindowSize(){return this->windowSize;};
		inline void SetPyramidLevel(int level=3){this->pyramidLevel = level;};
		inline int GetPyramidLevel(){return this->pyramidLevel;};

		void Initialize(int width, int height, CvSize windowSize=cvSize(10, 10), int pyramidLevel=3);
		int TrackFeature(IplImage* grayImage, std::vector<CvPoint2D32f>* prevPoints, std::vector<CvPoint2D32f>* currentPoints);
	};
}

#endif