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

#include "Algorithms/FLANNtree.h"
using namespace windage;
using namespace windage::Algorithms;

bool FLANNtree::Training(std::vector<windage::FeaturePoint>* pointList)
{
	if(pointList == NULL)
		return false;

	int count = (int)pointList->size();
	if(count <= 0)
		return false;

	int dimension = (*pointList)[0].DESCRIPTOR_DIMENSION;

	if(this->descriptorStorage) cvReleaseMat(&this->descriptorStorage);
	this->descriptorStorage = cvCreateMat(count, dimension, DESCRIPTOR_DATA_TYPE);
	
	for(int y=0; y<count; y++)
	{
		for(int x=0; x<dimension; x++)
		{
			CV_MAT_ELEM((*this->descriptorStorage), float, y, x) = (float)(*pointList)[y].descriptor[x];
		}
	}

	flannStorage = cv::Mat(descriptorStorage, false);
	if(this->flannIndex) delete flannIndex;
	this->flannIndex = new cv::flann::Index(flannStorage, cv::flann::KDTreeIndexParams(2));

	return true;
}

int FLANNtree::Matching(windage::FeaturePoint point, double* difference)
{
	int index = -1;

	int dimension = point.DESCRIPTOR_DIMENSION;
	CvMat* currentDescriptor = cvCreateMat(1, dimension, DESCRIPTOR_DATA_TYPE);
	cv::Mat resultIndex(1, 2, CV_32S);
	cv::Mat resultDistance(1, 2, CV_32FC1);

	for(int i=0; i<dimension; i++)
		CV_MAT_ELEM((*currentDescriptor), float, 0, i) = (float)point.descriptor[i];

	cv::Mat descriptor(currentDescriptor, false);
	this->flannIndex->knnSearch(descriptor, resultIndex, resultDistance, 2, cv::flann::SearchParams(this->eMax));
	
	double minDistance1 = (double)(resultDistance.ptr<float>(0)[0]);
	double minDistance2 = (double)(resultDistance.ptr<float>(0)[1]);
	int minIndex1 = resultIndex.ptr<int>(0)[0];
	int minIndex2 = resultIndex.ptr<int>(0)[1];
	
	index = minIndex1;
    if(minDistance2 < minDistance1)
    {
		double temp = minDistance1;
		minDistance1 = minDistance2;
		minDistance2 = temp;
		index = minIndex2;
    }

	cvReleaseMat(&currentDescriptor);

	if(difference)
		(*difference) = minDistance1;
	if(minDistance1 > minDistance2 * this->nearestNeighbourhoodRatio)
		return -1;
	return index;
}