/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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

#include "Algorithms/KDforest.h"
using namespace windage;
using namespace windage::Algorithms;

bool KDforest::Training(std::vector<windage::FeaturePoint>* pointList)
{
	if(pointList == NULL)
		return false;

	int count = (int)pointList->size();
	if(count <= 0)
		return false;

	int stepCount = cvRound((double)count / (double)this->treeNumber);
	int overlabCount = cvRound((double)stepCount * this->overlab);
	int dimension = (*pointList)[0].DESCRIPTOR_DIMENSION;
	
	for(int i=0; i<this->treeNumber; i++)
	{
		if(this->descriptorStorage[i]) cvReleaseMat(&this->descriptorStorage[i]);
		this->descriptorStorage[i] = cvCreateMat(stepCount + overlabCount, dimension, DESCRIPTOR_DATA_TYPE);

		this->descriptorIndex[i].resize(stepCount + overlabCount);
	}

	for(int i=0; i<count; i++)
	{
		int index = i%this->treeNumber;
		int y = (i-index)/this->treeNumber;
		this->descriptorIndex[index][y] = i;
		for(int x=0; x<dimension; x++)
		{
			CV_MAT_ELEM((*(this->descriptorStorage[index])), double, y, x) = (*pointList)[i].descriptor[x];
		}
	}

	CvRNG rng = cvRNG(cvGetTickCount());
	for(int i=0; i<overlabCount; i++)
	{
		for(int index=0; index<this->treeNumber; index++)
		{
			int randIndex = cvRandInt(&rng) % count;
			int y = stepCount + i;
			this->descriptorIndex[index][y] = randIndex;
			for(int x=0; x<dimension; x++)
			{
				CV_MAT_ELEM((*(this->descriptorStorage[index])), double, y, x) = (*pointList)[randIndex].descriptor[x];
			}
		}
	}

	for(int i=0; i<this->treeNumber; i++)
	{
		if(this->spilltree[i]) cvReleaseFeatureTree(this->spilltree[i]);
		this->spilltree[i] = cvCreateKDTree(this->descriptorStorage[i]);
	}
	
	return true;
}

int KDforest::Matching(windage::FeaturePoint point, double* difference)
{
	int index = -1;

	int dimension = point.DESCRIPTOR_DIMENSION;
	CvMat* currentDescriptor = cvCreateMat(1, dimension, DESCRIPTOR_DATA_TYPE);
	CvMat* resultIndex = cvCreateMat(1, 1, CV_32S);
	CvMat* resultDistance = cvCreateMat(1, 1, CV_64FC1);

	for(int i=0; i<dimension; i++)
		CV_MAT_ELEM((*currentDescriptor), double, 0, i) = point.descriptor[i];

	std::vector<int> indexList;
	std::vector<double> distanceList;

	for(int i=0; i<this->treeNumber; i++)
	{
		cvFindFeatures(this->spilltree[i], currentDescriptor, resultIndex, resultDistance, 1, this->eMax);
		indexList.push_back(CV_MAT_ELEM((*resultIndex), int, 0, 0));
		distanceList.push_back(CV_MAT_ELEM((*resultDistance), double, 0, 0));
	}
	
	double minDistance1=99999999, minDistance2=99999999;
	int minIndex1, minIndex2;

	for(int i=0; i<this->treeNumber; i++)
	{
		if(distanceList[i] < minDistance1)
		{
			minDistance2 = minDistance1;
			minIndex2 = minIndex1;

			minDistance1 = distanceList[i];
			minIndex1 = this->descriptorIndex[i][indexList[i]];
		}
		else if(distanceList[i] < minDistance2 && minIndex1 != this->descriptorIndex[i][indexList[i]])
		{
			minDistance2 = distanceList[i];
			minIndex2 = this->descriptorIndex[i][indexList[i]];
		}
	}
	index = minIndex1;

	cvReleaseMat(&currentDescriptor);
	cvReleaseMat(&resultIndex);
	cvReleaseMat(&resultDistance);

	if(difference)
		(*difference) = minDistance1;
	if(minDistance1 > minDistance2 * this->nearestNeighbourhoodRatio)
		return -1;

	return index;
}