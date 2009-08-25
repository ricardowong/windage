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

#include "SpatialInteraction/StereoSURFSpatialSensor.h"
using namespace windage;

StereoSURFSpatialSensor::StereoSURFSpatialSensor()
{
	cameraNumber = 0;
	activationThreshold = 0;

}

StereoSURFSpatialSensor::~StereoSURFSpatialSensor()
{
	this->Release();
}

void StereoSURFSpatialSensor::Release()
{
	this->cameraParameters.clear();
}

void StereoSURFSpatialSensor::Initialize(Vector3 position, double activationThreshold, int cameraNumber)
{
	this->Release();

	this->SetPosition(position);

	this->SetActivationThreshold(activationThreshold);
	this->SetCameraNumber(cameraNumber);

	this->cameraParameters.resize(cameraNumber);
}

void StereoSURFSpatialSensor::AttatchCameraParameter(int cameraNumber, Calibration* cameraParameters)
{
	if(cameraNumber < this->cameraParameters.size())
	{
		this->cameraParameters[cameraNumber] = cameraParameters;
	}
}

double StereoSURFSpatialSensor::GetDisparity(std::vector<IplImage*>* images)
{
	std::vector<CvPoint> featurePoints;
	std::vector<SURFDesciription> description;
	
	for(int i=0; i<this->cameraNumber; i++)
	{
		featurePoints.clear();
		CvPoint imageCoordinate = cameraParameters[i]->ConvertWorld2Image(this->position.x, this->position.y, this->position.z);
		featurePoints.push_back(imageCoordinate);

		std::vector<SURFDesciription> tempDescription;
		ModifiedSURFTracker::ExtractModifiedSURF((*images)[i], &featurePoints, &tempDescription);

		if((int)tempDescription.size() > 0)
			description.push_back(tempDescription[0]);
	}

	double disparity = 0.0;
	if((int)description.size() == this->cameraNumber)
		disparity = description[0].distance(description[1]);
	return disparity;
}

bool StereoSURFSpatialSensor::CalculateActivation(std::vector<IplImage*>* images)
{
	this->SetActivation(false);

	if((int)images->size() == this->cameraNumber && (int)cameraParameters.size() == this->cameraNumber)
	{
		double diaparity = this->GetDisparity(images);

		if(diaparity < this->GetActivationThreshold())
		{
			this->SetActivation(true);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
