
#include <cv.h>

#include "windageMatrix.h"

extern "C" __declspec(dllimport)
int FindChessBoardCorner(CvPoint2D32f* resultPoint, IplImage* image, int chessBoardWidth, int chessBoardHeight);
// CvPoint2D32f * result = new CvPoint2D32f[(chessBoardWidth-1) * (chessBoardHeight-1)];


extern "C" __declspec(dllimport)
/*** output ***/
// Matrix3 &intrinsicMatrix
// Vector4 &distortionCoefficients
// Vector3 rotationVector[imageCount]
// Vector3 translationVector[imageCount]
/*** input ***/
// CvPoint2D32f * cornersArray = new CvPoint2D32f[imageCount * (chessBoardWidth-1) * (chessBoardHeight-1)];
void SolveCalibration(Matrix3* intrinsicMatrix, Vector4* distortionCoefficients, Vector3* rotationVector, Vector3* translationVector,
					  CvPoint2D32f* cornersArray, int chessBoardWidth, int chessBoardHeight, double fieldSize,
					  int imageWidth, int imageHeight, int imageCount);

extern "C" __declspec(dllimport)
void GetExtrinsicMatrix(Matrix4* extrinsicMatrix, Vector3 rotationVector, Vector3 translationVector);

extern "C" __declspec(dllimport)
void UnRadialDistortion(IplImage* result, IplImage* input, Matrix3 instrinsicMatrix, Vector4 distortionCoefficients);