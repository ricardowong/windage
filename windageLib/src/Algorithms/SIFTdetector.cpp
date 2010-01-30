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

#include "Algorithms/SIFTdetector.h"
#include "Structures/SIFTpoint.h"
using namespace windage;
using namespace windage::Algorithms;

#include "Algorithms/SIFT/sift.h"
#include "Algorithms/SIFT/imgfeatures.h"

bool SIFTdetector::DoExtractKeypointsDescriptor(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;
	if(grayImage->nChannels != 1)
		return false;

	this->ResetKeypoints();
	
	struct feature* features = NULL;
	int count = sift_features(grayImage, &features);

	windage::SIFTpoint* point;
	for(int i=0; i<count; i++)
	{
		point = new windage::SIFTpoint();

		point->SetPoint(windage::Vector3(features[i].x, features[i].y, 1.0));
		point->SetSize(cvRound(features[i].scl * SIZE_AMPLIFICATION));
		point->SetDir(features[i].ori);

		for(int j=0; j<point->DESCRIPTOR_DIMENSION; j++)
		{
			point->descriptor[j] = features[i].descr[j];
		}

		this->keypoints.push_back((FeaturePoint*)point);
	}

	if(features) delete[] features;
	return true;
}
