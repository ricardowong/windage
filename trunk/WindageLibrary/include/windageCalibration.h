
#include <cv.h>
#include <vector>

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
// vector<Vector3>* objectCornersArray
// vector<Vector3>* imageCornersArray
// (objectCornersArray.size() == cornerCount * imageCount;
/*** description ***/
// atypical object based calibration
void SolveCalibration(Matrix3* intrinsicMatrix, Vector4* distortionCoefficients, Vector3* rotationVector, Vector3* translationVector,
					  std::vector<Vector3>* objectCornersArray, std::vector<Vector3>* imageCornersArray, int cornerCount,
					  int imageWidth, int imageHeight, int imageCount);

extern "C" __declspec(dllimport)
/*** output ***/
// Vector3 &rotationVector
// Vector3 &translationVector
/*** input ***/
// vector<Vector3>* objectCornersArray
// vector<Vector3>* imageCornersArray
// (objectCornersArray.size() == cornerCount * imageCount;
/*** description ***/
// atypical object based calibration
void UpdateExtrinsicParams(Vector3* rotationVector, Vector3* translationVector,
					  Matrix3 intrinsicMatrix, Vector4 distortionCoefficients,
					  std::vector<Vector3>* objectCornersArray, std::vector<Vector3>* imageCornersArray, int cornerCount);

extern "C" __declspec(dllimport)
/*** output ***/
// Matrix3 &intrinsicMatrix
// Vector4 &distortionCoefficients
// Vector3 rotationVector[imageCount]
// Vector3 translationVector[imageCount]
/*** input ***/
// CvPoint2D32f * cornersArray = new CvPoint2D32f[imageCount * (chessBoardWidth-1) * (chessBoardHeight-1)];
/*** description ***/
// Checker board based calibration
void SolveCalibration2(Matrix3* intrinsicMatrix, Vector4* distortionCoefficients, Vector3* rotationVector, Vector3* translationVector,
					  CvPoint2D32f* cornersArray, int chessBoardWidth, int chessBoardHeight, double fieldSize,
					  int imageWidth, int imageHeight, int imageCount);

extern "C" __declspec(dllimport)
/*** output ***/
// Vector3 &rotationVector
// Vector3 &translationVector
/*** input ***/
// Matrix3 intrinsicMatrix
// Vector4 distortionCoefficients
// CvPoint2D32f * cornersArray = new CvPoint2D32f[(chessBoardWidth-1) * (chessBoardHeight-1)];
/*** description ***/
// Checker board based calibration
void UpdateExtrinsicParams2(Vector3* rotationVector, Vector3* translationVector,
					  Matrix3 intrinsicMatrix, Vector4 distortionCoefficients,
					  CvPoint2D32f* cornersArray, int chessBoardWidth, int chessBoardHeight, double fieldSize,
					  int imageWidth, int imageHeight);

extern "C" __declspec(dllimport)
void GetExtrinsicMatrix(Matrix4* extrinsicMatrix, Vector3 rotationVector, Vector3 translationVector);

extern "C" __declspec(dllimport)
void UnRadialDistortion(IplImage* result, IplImage* input, Matrix3 instrinsicMatrix, Vector4 distortionCoefficients);