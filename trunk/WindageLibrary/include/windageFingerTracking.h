
#include <cv.h>
#include <vector>

#include "windageVector.h"

extern "C" __declspec(dllimport)
void SkinDetection(IplImage* resultImage, IplImage* inputImage,
				   int cbThresholdU=127, int cbThresholdL=77, int crThresholdU=173, int crThresholdL=133);

extern "C" __declspec(dllimport)
double HandDetection(IplImage* result, Vector3* centerPosition,  IplImage* input);

extern "C" __declspec(dllexport)
void ConvexHull(std::vector<Vector3>* pointList, IplImage* input, double distanceThreshold, IplImage* debugImage = NULL);

extern "C" __declspec(dllexport)
void FindFingerTipPoint(std::vector<Vector3>* result, std::vector<Vector3>* pointList,
						Vector3 centerPoint, double radius, int border, IplImage* inputImage, IplImage* debugImage = NULL);
