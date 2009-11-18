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

#include "SpatialInteraction/StereoSURFDetector.h"
using namespace windage;

StereoSURFDetector::StereoSURFDetector()
{
	cameraNumber = 0;
	activationThreshold = 0;

}

StereoSURFDetector::~StereoSURFDetector()
{
	this->Release();
}

void StereoSURFDetector::Release()
{
	this->cameraParameters.clear();
}

void StereoSURFDetector::Initialize(double activationThreshold, int cameraNumber)
{
	this->Release();

	this->SetActivationThreshold(activationThreshold);
	this->SetCameraNumber(cameraNumber);

	this->cameraParameters.resize(cameraNumber);
}

void StereoSURFDetector::AttatchCameraParameter(int cameraNumber, Calibration* cameraParameters)
{
	if(cameraNumber < (int)this->cameraParameters.size())
	{
		this->cameraParameters[cameraNumber] = cameraParameters;
	}
}

double StereoSURFDetector::GetDisparity(std::vector<IplImage*>* images, SpatialSensor* sensor)
{
	std::vector<CvPoint> featurePoints;
	std::vector<SURFDesciription> description;
	
	for(int i=0; i<this->cameraNumber; i++)
	{
		Vector3 position = sensor->GetPosition();

		featurePoints.clear();
		CvPoint imageCoordinate = cameraParameters[i]->ConvertWorld2Image(position.x, position.y, position.z);
		featurePoints.push_back(imageCoordinate);

		std::vector<SURFDesciription> tempDescription;
		ModifiedSURFTracker::ExtractModifiedSURF((*images)[i], &featurePoints, &tempDescription);

		if((int)tempDescription.size() > 0)
			description.push_back(tempDescription[0]);
	}

	double disparity = 0.0;
	if((int)description.size() == this->cameraNumber)
		disparity = description[0].getDistance(description[1]);
	return disparity;
}

void StereoSURFDetector::CalculateActivation(std::vector<IplImage*>* images)
{
	for(int i=0; i<(int)this->spatialSensors->size(); i++)
	{
		(*spatialSensors)[i]->SetInactive();
		double diaparity = this->GetDisparity(images, (*spatialSensors)[i]);
		if(diaparity < this->GetActivationThreshold())
		{
			(*spatialSensors)[i]->SetActive();
		}
	}
}
