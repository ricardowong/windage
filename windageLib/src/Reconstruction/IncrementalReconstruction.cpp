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

#include "Reconstruction/IncrementalReconstruction.h"
using namespace windage;
using namespace windage::Reconstruction;

#include "Reconstruction/BundleWrapper.h"

bool IncrementalReconstruction::BundleAdjustment()
{
	BundleWrapper* bundler = new windage::Reconstruction::BundleWrapper();



	return false;
}

void IncrementalReconstruction::AttatchFeaturePoint(std::vector<windage::FeaturePoint>* featurePoints)
{
	this->sceneCount++;
	this->cameraParameters.resize(this->sceneCount);
	this->reconstructionPoints.resize(this->sceneCount);
	this->featurePointsList.resize(this->sceneCount);

	double fx = this->initialCameraParameter->GetParameters()[0];
	double fy = this->initialCameraParameter->GetParameters()[1];
	double cx = this->initialCameraParameter->GetParameters()[2];
	double cy = this->initialCameraParameter->GetParameters()[3];

	this->cameraParameters[this->sceneCount-1] = new windage::Calibration();
	this->cameraParameters[this->sceneCount-1]->Initialize(fx, fy, cx, cy);
	for(unsigned int i=0; i<featurePoints->size(); i++)
	{
		this->featurePointsList[this->sceneCount-1].push_back((*featurePoints)[i]);
	}
}

bool IncrementalReconstruction::Calculate()
{
	if(this->sceneCount <= 0)
		return false;
	if(this->initialCameraParameter == NULL)
		return false;

	return true;
}