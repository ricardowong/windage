/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo
 *   U-VR Lab, GIST of Gwangju in Korea.
 *   http://windage.googlecode.com/
 *   http://uvr.gist.ac.kr/
 *
 * Copyright of the derived and new portions of this work
 *     (C) 2009 GIST U-VR Lab.
 *
 * This framework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information please contact 
 *   Woonhyuk Baek
 *   <windage@live.com>
 *   GIST U-VR Lab.
 *   Department of Information and Communication
 *   Gwangju Institute of Science and Technology
 *   1, Oryong-dong, Buk-gu, Gwangju
 *   South Korea
 * ========================================================================
 ** @author   Woonhyuk Baek
 * ======================================================================== */

#include "Tracker/ReferenceStorage.h"
#include "Tracker/ModifiedSURFTracker.h"

using namespace windage;

void SURFReferenceStorage::Release()
{
	if(referenceFeatureStorage) cvReleaseMat(&referenceFeatureStorage);
	if(flannIndex) delete flannIndex;
	if(referenceImage) cvReleaseImage(&referenceImage);
}

int SURFReferenceStorage::GenerateReferenceFeatureTree(double scaleFactor, int scaleStep)
{
	int width = (int)((double)referenceImage->width/scaleFactor);
	int height = (int)((double)referenceImage->height/scaleFactor);

	descriptor.clear();

	IplImage* tempReference = NULL;
	for(int y=1; y<=scaleStep; y++)
	{
		for(int x=1; x<=scaleStep; x++)
		{
			tempReference = cvCreateImage(cvSize(width*x, height*y), IPL_DEPTH_8U, 1);
			cvResize(referenceImage, tempReference, CV_INTER_LINEAR);
			cvSmooth(tempReference, tempReference, CV_GAUSSIAN, 3, 3);

			std::vector<SURFDesciription> tempSurf;
			tempSurf.clear();

			std::vector<CvPoint> fastCorners;
			ModifiedSURFTracker::ExtractFASTCorner(&fastCorners, tempReference, featureExtractThreshold);
			ModifiedSURFTracker::ExtractModifiedSURF(tempReference, &fastCorners, &tempSurf);
			float xScaleFactor = (float)this->realWidth / (float)tempReference->width;
			float yScaleFactor = (float)this->realHeight / (float)tempReference->height;

			for(int i=0; i<(int)tempSurf.size(); i++)
			{
				tempSurf[i].point.x *= xScaleFactor;
				tempSurf[i].point.x += xScaleFactor/2.0f;
				tempSurf[i].point.y *= yScaleFactor;
				tempSurf[i].point.y += yScaleFactor/2.0f;
				tempSurf[i].point.y = (float)this->realHeight - tempSurf[i].point.y;

				tempSurf[i].point.x = tempSurf[i].point.x - (float)this->realWidth/2;
				tempSurf[i].point.y = tempSurf[i].point.y - (float)this->realHeight/2;

				descriptor.push_back(tempSurf[i]);
			}

			cvReleaseImage(&tempReference);
		}
	}

	this->CreateFlannTree(&this->descriptor, referenceFeatureStorage);
	return 0;
}

bool SURFReferenceStorage::CreateFlannTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage)
{
	int count = (int)referenceSURF->size();

	if(referenceFeatureStorage != NULL) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = cvCreateMat(count, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);

	for(int y=0; y<count; y++)
	{
		for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
		{
			cvmSet(referenceFeatureStorage, y, x, (*referenceSURF)[y].descriptor[x]);
		}
	}

	cv::Mat refer(referenceFeatureStorage, false);
	flannFeatureTree = refer;

	if(flannIndex) delete flannIndex;
	flannIndex = new cv::flann::Index(flannFeatureTree, cv::flann::KDTreeIndexParams(2));
	return true;
}

void SURFReferenceStorage::RegistReferenceImage(IplImage* referenceImage, double realWidth, double realHeight, double scaleFactor, int scaleStep)
{
	this->Release();
	this->referenceImage = cvCloneImage(referenceImage);
	this->realWidth = cvRound(realWidth);
	this->realHeight = cvRound(realHeight);

	GenerateReferenceFeatureTree(scaleFactor, scaleStep);

}

int SURFReferenceStorage::FindPairs(SURFDesciription description, float distanceRate, int EMAX, float* outDistance)
{
	CvMat* currentFeature = cvCreateMat(1, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);
	cv::Mat result(1, 2, CV_32S);
	cv::Mat distance(1, 2, SURF_DESCRIPTOR_TYPE);

	for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
	{
		cvmSet(currentFeature, 0, x, description.descriptor[x]);
	}

	cv::Mat object(currentFeature, false);
	flannIndex->knnSearch(object, result, distance, 2, cv::flann::SearchParams(EMAX));

	float min2 = distance.ptr<float>(0)[0];
	float min1 = distance.ptr<float>(0)[1];
	int index2 = result.ptr<int>(0)[0];
	int index1 = result.ptr<int>(0)[1];
	int index = index1;
//*
	float temp;
	if(min2 < min1)
	{
		temp = min1;
		min1 = min2;
		min2 = temp;
		index = index2;
	}
//*/
	cvReleaseMat(&currentFeature);

	if(outDistance)
		(*outDistance) = (float)min2;

	if ( min1 < distanceRate*min2 )
		return index;
    return -1;
}

