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

#include "Frameworks/ObjectTracking.h"
using namespace windage;
using namespace windage::Frameworks;

bool ObjectTracking::AttatchReferenceImage(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;

	if(this->referenceImage) cvReleaseImage(&this->referenceImage);
	this->referenceImage = NULL;

	this->referenceImage = cvCloneImage(grayImage);
	return true;
}

bool ObjectTracking::Initialize(int width, int height)
{
	this->width = width;
	this->height = height;

	this->detector->DoExtractKeypointsDescriptor(this->referenceImage);
	std::vector<windage::FeaturePoint>* tempReferenceImage = this->detector->GetKeypoints();

	this->referenceRepository.clear();
	for(int i=0; i<tempReferenceImage->size(); i++)
	{
		this->referenceRepository.push_back((*tempReferenceImage)[i]);
	}
	
	return true;
}

bool ObjectTracking::UpdateCamerapose(IplImage* grayImage)
{
	this->detector->DoExtractKeypointsDescriptor(grayImage);
	std::vector<windage::FeaturePoint>* sceneKeypoints = this->detector->GetKeypoints();



	return true;
}