#include <vector>
#include <windage.h>

using namespace windage;
CvFeatureTree* CreateReferenceTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage)
{
	int count = referenceSURF->size();
	for(int y=0; y<count; y++)
	{
		for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
		{
			cvSetReal2D(referenceFeatureStorage, y, x, (*referenceSURF)[y].descriptor[x]);
		}
	}
	CvFeatureTree* tree = cvCreateKDTree(referenceFeatureStorage);
	return tree;
}

int FindPairs(SURFDesciription description, CvFeatureTree* tree, double distanceRate)
{
	CvMat* currentFeature = cvCreateMat(1, SURF_DESCRIPTOR_DIMENSION, CV_32FC1);
	CvMat* result = cvCreateMat(1, 2, CV_32SC1);
	CvMat* distance = cvCreateMat(1, 2, CV_64FC1);
	for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
	{
		cvSetReal2D(currentFeature, 0, x, description.descriptor[x]);
	}

	cvFindFeatures(tree, currentFeature, result, distance, 2, 20);

	double min2 = cvGetReal2D(distance, 0, 0);
	double min1 = cvGetReal2D(distance, 0, 1);
	int index2 = (int)cvGetReal2D(result, 0, 0);
	int index1 = (int)cvGetReal2D(result, 0, 1);
	int index = index1;
//*
	double temp;
	if(min2 < min1)
	{
		temp = min1;
		min1 = min2;
		min2 = temp;
		index = index2;
	}
//*/
	cvReleaseMat(&currentFeature);
	cvReleaseMat(&result);
	cvReleaseMat(&distance);

	if ( min1 < distanceRate*min2 )
		return index;
    return -1;
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