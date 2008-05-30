
#include <cv.h>
#include <vector>

#include "windageMatrix.h"


extern "C" __declspec(dllimport)
void ImageToWorldCoordinate(Vector3* result, Vector3 point, Matrix3 instrinsicMatrix, Matrix3 rotationMatrix, Vector3 translateVector);

extern "C" __declspec(dllimport)
void WorldToImageCoordinate(Vector3* result, Vector3 point, Matrix3 instrinsicMatrix, Matrix3 rotationMatrix, Vector3 translateVector);

extern "C" __declspec(dllimport)
void WorldCoordinateCrossPointApproximation(Vector3* result, Vector3* point1, Vector3* point2,
							  double distanceThreshold, Matrix3 k, Matrix3 r1, Matrix3 r2, Vector3 t1, Vector3 t2, Vector3 c1, Vector3 c2);

extern "C" __declspec(dllimport)
void FindWorldCoordinatePointApproximation(std::vector<Vector3>* result, std::vector<Vector3>* pointList1, std::vector<Vector3>* pointList2,
							  double distanceThreshold, Matrix3 k, Matrix3 r1, Matrix3 r2, Vector3 t1, Vector3 t2, Vector3 c1, Vector3 c2,
							  IplImage* debugImage = NULL);
