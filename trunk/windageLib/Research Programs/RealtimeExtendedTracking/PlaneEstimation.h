#include <windage.h>

using namespace windage;
Vector4 PlaneEstimation(Vector3 point1, Vector3 point2, Vector3 point3)
{
	Vector3 direction1 = point2 - point1;
	Vector3 direction2 = point3 - point1;
	
	Vector3 plane = direction1^direction2;
	double d = - (plane * point1);

	return Vector4(plane.x, plane.y, plane.z, d);
}

double CalculateError(Vector4 plane, Vector3 point)
{
	Vector3 normal = Vector3(plane.x, plane.y, plane.z);
	double error = abs(normal * point + plane.w) / normal.getLength();
	return error;
}

#include <time.h>
Vector4 PlaneEstimationRANSAC(std::vector<Vector3>* points, std::vector<Vector3>* consensusPoints, Vector3& center)
{
	srand(time(NULL));
	Vector4 bestResult;

	Vector3 bestSum;
	double bestCount = 0;
	double bestError = 9999999;
	Vector4 bestPlane;

	for(int i=0; i<points->size() * 100; i++)
	{
		int index[3];
		
		index[0] = rand()%points->size();
		index[1] = rand()%points->size();
		index[2] = rand()%points->size();

		while(index[0] == index[1] || index[0] == index[2] || index[1] == index[2])
		{
			index[0] = rand()%points->size();
			index[1] = rand()%points->size();
			index[2] = rand()%points->size();
		}

		Vector3 point1 = (*points)[index[0]];
		Vector3 point2 = (*points)[index[1]];
		Vector3 point3 = (*points)[index[2]];

		Vector4 plane = PlaneEstimation(point1, point2, point3);

		const double ERROR_THRESHOLD = 1.0;

		Vector3 sum = Vector3();
		int count = 0;
		double error = 0.0;
		for(int j=0; j<points->size(); j++)
		{
			double tempError = CalculateError(plane, (*points)[j]);
			if(tempError < ERROR_THRESHOLD)
			{
				error += tempError;
				count++;
				sum += (*points)[j];
			}
		}
		error /= count;
		sum /= count;

		if(count >= bestCount)
		{
			if(count == bestCount)
			{
				if(error < bestError)
				{
					bestPlane = plane;
					bestCount = count;
					bestError = error;
					bestSum = sum;

					consensusPoints->clear();
					for(int j=0; j<points->size(); j++)
					{
						double tempError = CalculateError(plane, (*points)[j]);
						if(tempError < ERROR_THRESHOLD)
						{
							consensusPoints->push_back((*points)[j]);
						}
					}
				}
			}
			else
			{
				bestPlane = plane;
				bestCount = count;
				bestError = error;
				bestSum = sum;

				consensusPoints->clear();
				for(int j=0; j<points->size(); j++)
				{
					double tempError = CalculateError(plane, (*points)[j]);
					if(tempError < ERROR_THRESHOLD)
					{
						consensusPoints->push_back((*points)[j]);
					}
				}
			}			
		}
	}

	center = bestSum;
	return bestPlane;
}

Matrix4 QuatToMatrix(Vector4 quat)
{
	double v1 = quat.x;
	double v2 = quat.y;
	double v3 = quat.z;
	double theta = quat.w;

    double t1 =  cos(theta);
	double t2 =  1 - t1;
	double t3 =  v1*v1;
	double t6 =  t2*v1;
	double t7 =  t6*v2;
	double t8 =  sin(theta);
	double t9 =  t8*v3;
	double t11 = t6*v3;
	double t12 = t8*v2;
	double t15 = v2*v2;
	double t19 = t2*v2*v3;
	double t20 = t8*v1;
	double t24 = v3*v3;

	Matrix4 matrix;
    matrix.m[0][0] = t1 + t2*t3;
	matrix.m[1][0] = t7 - t9;
    matrix.m[2][0] = t11 + t12;
	matrix.m[3][0] = 0.0;

    matrix.m[0][1] = t7 + t9;
	matrix.m[1][1] = t1 + t2*t15;
    matrix.m[2][1] = t19 - t20;
	matrix.m[3][1] = 0.0;

    matrix.m[0][2] = t11 - t12;
	matrix.m[1][2] = t19 + t20;
    matrix.m[2][2] = t1 + t2*t24;
	matrix.m[3][2] = 0.0;

    matrix.m[0][3] = 0.0;
	matrix.m[1][3] = 0.0;
    matrix.m[2][3] = 0.0;
	matrix.m[3][3] = 1.0;
	return matrix;
}

double Radian2Degree(double radian)
{
	const double PI = 3.14159265;
	return radian * (180.0 / PI);
}

Vector3 ConvertWorld2PlaneCoordinate(Vector3 point, Vector3 normal)
{
	Vector3 zAxis = Vector3(0.0, 0.0, 1.0);

	Vector3 rotationAxis = zAxis^normal;
//	rotationAxis /= rotationAxis.getLength();
	double theta = acos((normal * zAxis) / (normal.getLength() * zAxis.getLength()));
//	theta = Radian2Degree(theta);

	Vector4 quaternion = Vector4(rotationAxis.x, rotationAxis.y, rotationAxis.z, theta);
	Matrix4 convert = QuatToMatrix(quaternion);

	Vector4 hPoint = Vector4(point.x, point.y, point.z, 1.0);
	Vector4 tempResult = convert * hPoint;
	tempResult /= tempResult.w;

	return Vector3(tempResult.x, tempResult.y, tempResult.z);
}

Vector3 ConvertWorld2PlaneCoordinate(Vector3 point, Vector3 rotationAxis, double theta)
{
	Vector4 quaternion = Vector4(rotationAxis.x, rotationAxis.y, rotationAxis.z, theta);
	Matrix4 convert = QuatToMatrix(quaternion);

	Vector4 hPoint = Vector4(point.x, point.y, point.z, 1.0);
	Vector4 tempResult = convert * hPoint;
	tempResult /= tempResult.w;

	return Vector3(tempResult.x, tempResult.y, tempResult.z);
}

void ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector, CvMat* extrinsicMatrix)
{
	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);

	cvSetZero(extrinsicMatrix);
	int i, j;
    for(i=0; i < 3; i++) 
    {
		cvSetReal2D(extrinsicMatrix, i, 3, cvGetReal1D(translationVector, i));
    } 

	cvRodrigues2(rotationVector, rotationMatrix);
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			cvSetReal2D(extrinsicMatrix, i, j, cvGetReal2D(rotationMatrix, i, j)); 
		}
	}

	cvSetReal2D(extrinsicMatrix, 3, 3, 1.0);

	cvReleaseMat(&rotationMatrix);
}

double CalculatePose(CvMat* intrinsic, CvMat* distortion, CvMat* extrinsicMatrix, std::vector<Vector2>* imagePoints, std::vector<Vector3>* objectPoints)
{
	CvMat* rotationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	int pointCount = imagePoints->size();

	CvMat* pImagePoints = cvCreateMat(pointCount, 2, CV_64FC1);
	CvMat* pObjectPoints = cvCreateMat(pointCount, 3, CV_64FC1);

	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, (*imagePoints)[i].x);
		cvSetReal2D(pImagePoints, i, 1, (*imagePoints)[i].y);
	}
	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pObjectPoints, i, 0, (*objectPoints)[i].x);
		cvSetReal2D(pObjectPoints, i, 1, (*objectPoints)[i].y);
		cvSetReal2D(pObjectPoints, i, 2, (*objectPoints)[i].z);
	}

	cvFindExtrinsicCameraParams2(pObjectPoints, pImagePoints, intrinsic, distortion, rotationVector, translationVector);
	ConvertExtrinsicParameter(rotationVector, translationVector, extrinsicMatrix);

	// release matrix
	if(pImagePoints)		cvReleaseMat(&pImagePoints);
	if(pObjectPoints)		cvReleaseMat(&pObjectPoints);
	if(rotationVector)		cvReleaseMat(&rotationVector);
	if(translationVector)	cvReleaseMat(&translationVector);

	return true;
}