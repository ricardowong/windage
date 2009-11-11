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

#include "FeatureMatching/FeatureTree.h"

using namespace windage;
FeatureTree::FeatureTree()
{
	referenceFeatureStorage = NULL;
	referenceFeatureTree = NULL;
}

FeatureTree::~FeatureTree()
{
	this->Release();
}

void FeatureTree::Release()
{
	if(referenceFeatureStorage) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = NULL;
	if(referenceFeatureTree) cvReleaseFeatureTree(referenceFeatureTree);
	referenceFeatureTree = NULL;
}

void FeatureTree::CreateReferenceTree(std::vector<SURFFeature*>* referenceSURF)
{
	this->Release();

	int count = 0;
	for(int i=0; i<(int)referenceSURF->size(); i++)
	{
		count += (*referenceSURF)[i]->GetFeatureCount();
	}
	referenceFeatureStorage = cvCreateMat(count, SURF_FEATURE_DESCRIPTOR_DIMENSION, CV_32FC1);

	int featureCount = 0;
	for(int y=0; y<referenceSURF->size(); y++)
	{
		for(int i=0; i<(*referenceSURF)[y]->GetFeatureCount(); i++)
		{
			for(int x=0; x<SURF_FEATURE_DESCRIPTOR_DIMENSION; x++)
			{
				cvSetReal2D(referenceFeatureStorage, featureCount, x, (*referenceSURF)[y]->GetDescription(i).descriptor[x]);
			}
			featureCount++;
		}
	}

	
	this->referenceFeatureTree = cvCreateKDTree(referenceFeatureStorage);
}

int FeatureTree::FindPairs(SURFFeature* description, double distanceRate)
{
	CvMat* currentFeature = cvCreateMat(1, SURF_FEATURE_DESCRIPTOR_DIMENSION, CV_32FC1);
	CvMat* result = cvCreateMat(1, 2, CV_32SC1);
	CvMat* distance = cvCreateMat(1, 2, CV_64FC1);
	for(int x=0; x<SURF_FEATURE_DESCRIPTOR_DIMENSION; x++)
	{
		cvSetReal2D(currentFeature, 0, x, description->GetDescription(0).descriptor[x]);
	}

	cvFindFeatures(this->referenceFeatureTree, currentFeature, result, distance, 2, 20);

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