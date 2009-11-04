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

#include "MultipleTracker.h"

using namespace windage;

void MultipleTracker::Release()
{
	while(referenceImageList.size() > 0)
	{
		this->DeleteReferenceImage(0);
	}
	delete opticalflow;
}
/*
void MultipleTracker::AttatchReferenceImage(IplImage* image)
{
	IplImage* temp = cvCloneImage(image);
	referenceImageList.push_back(temp);

	double* params = GetCameraParameter()->GetParameters();
	windage::Calibration* tempCalibration = new windage::Calibration();
	tempCalibration->Initialize(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7]);
	cameraParameterList.push_back(tempCalibration);
}

bool MultipleTracker::DeleteReferenceImage(int index)
{
	if(index < referenceImageList.size())
	{
		if(referenceImageList[index]) cvReleaseImage(&referenceImageList[index]);
		referenceImageList.erase(referenceImageList.begin()+index);

		delete cameraParameterList[index];
		cameraParameterList.erase(cameraParameterList.begin()+index);
		return true;
	}
	else
	{
		return false;
	}
};
*/
Calibration* MultipleTracker::GetCameraParameter(int index)
{
	if(index < 0)
	{
		return this->cameraParameter;
	}
	else if(index < cameraParameterList.size())
	{
		return this->cameraParameterList[index];
	}
	else
	{
		return NULL;
	}
};

void MultipleTracker::InitializeOpticalFlow(int width, int height, CvSize windowSize, int pyramidLevel)
{
	if(opticalflow) delete opticalflow;
	opticalflow = new OpticalFlow();
	opticalflow->Initialize(width, height, windowSize, pyramidLevel);
	opticalflow->SetRemovePrevPoints(false);
}