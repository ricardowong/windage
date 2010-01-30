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

	this->detector->DoExtractKeypointsDescriptor(this->referenceImage);
	std::vector<windage::FeaturePoint>* tempReferenceImage = this->detector->GetKeypoints();

	this->referenceRepository.clear();
	for(unsigned int i=0; i<tempReferenceImage->size(); i++)
	{
		windage::Vector3 point = (*tempReferenceImage)[i].GetPoint();
		point.x -= (double)width/2.0;
		point.y = (double)height/2.0 - point.y + 1;

		(*tempReferenceImage)[i].SetPoint(point);
		this->referenceRepository.push_back((*tempReferenceImage)[i]);
	}

	this->matcher->Training(&this->referenceRepository);

	return true;
}

bool ObjectTracking::Initialize(int width, int height)
{
	if(this->cameraParameter == NULL)
		return false;
	
	this->width = width;
	this->height = height;

	this->estimator->AttatchCameraParameter(this->cameraParameter);

	return true;
}

bool ObjectTracking::UpdateCamerapose(IplImage* grayImage)
{
	this->detector->DoExtractKeypointsDescriptor(grayImage);
	std::vector<windage::FeaturePoint>* sceneKeypoints = this->detector->GetKeypoints();

	std::vector<windage::FeaturePoint> refMatchedKeypoints;
	std::vector<windage::FeaturePoint> sceMatchedKeypoints;

	for(unsigned int i=0; i<sceneKeypoints->size(); i++)
	{
		int index = this->matcher->Matching((*sceneKeypoints)[i]);

		if(index >= 0)
		{
			refMatchedKeypoints.push_back(this->referenceRepository[index]);
			sceMatchedKeypoints.push_back((*sceneKeypoints)[i]);
		}
	}

	estimator->AttatchReferencePoint(&refMatchedKeypoints);
	estimator->AttatchScenePoint(&sceMatchedKeypoints);
	estimator->Calculate();

	return true;
}

void ObjectTracking::DrawOutLine(IplImage* colorImage, bool drawCross)
{
	int size = 4;
	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(255, 255, 255);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	color2, 6);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	color, 2);

	if(drawCross)
	{
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	color2, 6);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	color2, 6);

		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, -this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, +this->height/2, 0.0),	color, 2);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->width/2, +this->height/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->width/2, -this->height/2, 0.0),	color, 2);
	}
}