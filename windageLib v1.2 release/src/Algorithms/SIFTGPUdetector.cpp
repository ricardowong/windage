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

#include "Algorithms/SIFTGPUdetector.h"
#include "Structures/SIFTpoint.h"
using namespace windage;
using namespace windage::Algorithms;

#include <GL/glut.h>

bool SIFTGPUdetector::DoExtractKeypointsDescriptor(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;
	if(grayImage->nChannels != 1)
		return false;

	const int DESCRIPTOR_DIMENSION = 128;
	this->keypoints.clear();

	sift->_dog_threshold = (float)this->threshold;
	
	std::vector<SiftGPU::SiftKeypoint> keypoints;
	std::vector<float> descriptors;

	int count = 0;
	if(sift->RunSIFT(grayImage->width, grayImage->height, grayImage->imageData, GL_LUMINANCE, GL_UNSIGNED_BYTE))
	{
		count = sift->GetFeatureNum()+1;
		keypoints.resize(count);
		descriptors.resize(count * DESCRIPTOR_DIMENSION);
		sift->GetFeatureVector(&keypoints[0], &descriptors[0]);
	}
	else
	{
		return false;
	}

	windage::SIFTpoint point;
	for(int i=0; i<count; i++)
	{
		point.SetPoint(windage::Vector3((double)keypoints[i].x, (double)keypoints[i].y, 1.0));
		point.SetSize((double)cvRound(keypoints[i].s));
		point.SetDir((double)keypoints[i].o);

		for(int j=0; j<point.DESCRIPTOR_DIMENSION; j++)
		{
			point.descriptor[j] = (double)descriptors[i*DESCRIPTOR_DIMENSION + j];
		}

		this->keypoints.push_back(point);
	}

	return true;
}
