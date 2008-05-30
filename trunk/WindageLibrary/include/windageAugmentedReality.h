#include <cv.h>

#include "windageMatrix.h"

extern "C" __declspec(dllimport)
void DrawBackgroundTexture(IplImage* image, int imageWidth=512, bool isFlip=false);

extern "C" __declspec(dllimport)
void SetOpenGLCalibrationData(Matrix3 internalMatrix, Matrix3 rotationMatrix, Vector3 translateionVector, int imageWidth, int imageHeight, bool flip);

