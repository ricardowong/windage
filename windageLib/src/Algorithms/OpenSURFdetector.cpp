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

#include "Algorithms/OpenSURFdetector.h"
#include "Structures/OpenSURFpoint.h"

#include "Algorithms/OpenSURF/surflib.h"

using namespace windage;
using namespace windage::Algorithms;

bool OpenSURFdetector::DoExtractKeypointsDescriptor(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;
	if(grayImage->nChannels != 1)
		return false;

	this->keypoints.clear();

	IpVec ipts;
	surfDetDes(grayImage, ipts, false, 5, 4, 2, this->threshold);

	Ipoint *ipt;
	windage::OpenSURFpoint point;
	for(unsigned int i = 0; i < ipts.size(); i++) 
	{
		ipt = &ipts.at(i);

		point.SetPoint(windage::Vector3(ipt->x, ipt->y, 1.0));
		point.SetSize(2.5f * ipt->scale);
		point.SetDir(-ipt->orientation);
		
		for(int j=0; j<point.DESCRIPTOR_DIMENSION; j++)
		{
			point.descriptor[j] = ipt->descriptor[j];
		}

		this->keypoints.push_back(point);
	}

	return true;
}
