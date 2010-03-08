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

#include <vector>

#include "Algorithms/WSURFMultidetector.h"
#include "Structures/WSURFpoint.h"

#include "Algorithms/windageSURF/fast.h"
#include "Algorithms/windageSURF/wsurf.h"
#include "Algorithms/windageSURF/wfastsurf.h"

#include "Algorithms/WSURFdetector.h"

using namespace windage;
using namespace windage::Algorithms;

bool WSURFMultidetector::DoExtractKeypointsDescriptor(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;
	if(grayImage->nChannels != 1)
		return false;

	this->keypoints.clear();

	windage::Algorithms::WSURFdetector* singleDetector = new windage::Algorithms::WSURFdetector();
	singleDetector->SetThreshold(this->threshold);

	for(unsigned int i=0; i<this->resizeImage.size(); i++)
	{
		cvResize(grayImage, this->resizeImage[i]);
		singleDetector->DoExtractKeypointsDescriptor(this->resizeImage[i]);

		std::vector<windage::FeaturePoint>* singlePoints = singleDetector->GetKeypoints();
		for(unsigned int j=0; j<singlePoints->size(); j++)
		{
			(*singlePoints)[j].SetSize(this->size[i]);
			windage::Vector3 pt = (*singlePoints)[j].GetPoint();
			pt.x *= this->xScale[i];
			pt.y *= this->yScale[i];
			(*singlePoints)[j].SetPoint(pt);

			this->keypoints.push_back((*singlePoints)[j]);
		}
	}

	delete singleDetector;

	return true;
}
