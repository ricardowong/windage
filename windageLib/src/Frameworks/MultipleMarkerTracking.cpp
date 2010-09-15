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

#include "Frameworks/MultipleMarkerTracking.h"
using namespace windage;
using namespace windage::Frameworks;

bool MultipleMarkerTracking::Initialize(int width,					///< input image width
							int height,					///< input image height
							bool printInfo
							)
{
	if(this->initialCamearParameter == NULL)
		return false;
	
	this->width = width;
	this->height = height;

	if(printInfo)
	{
		std::cout << this->GetFunctionName() << " Initialize" << std::endl;
		if(this->estimator)
			std::cout << "\tPose Estimation : " << this->estimator->GetFunctionName() << std::endl;
		if(this->refiner)
			std::cout << "\tPose Refiner : " << this->refiner->GetFunctionName() << std::endl;
		std::cout << std::endl;
	}

	this->initialize = true;
	return true;
}

bool MultipleMarkerTracking::AttatchChessboard(int width, int height, double size)
{
	chessParameter.push_back(windage::Vector3(width, height, size));

	int objectCount = (int)chessParameter.size();
	this->cameraParameter.resize(objectCount);
	this->cameraParameter[objectCount-1] = new windage::Calibration();
	this->cameraParameter[objectCount-1]->Initialize(	this->initialCamearParameter->GetParameters()[0],
															this->initialCamearParameter->GetParameters()[1],
															this->initialCamearParameter->GetParameters()[2],
															this->initialCamearParameter->GetParameters()[3],
															this->initialCamearParameter->GetParameters()[4],
															this->initialCamearParameter->GetParameters()[5],
															this->initialCamearParameter->GetParameters()[6],
															this->initialCamearParameter->GetParameters()[7]);

	this->detector.resize(objectCount);
	this->detector[objectCount-1] = new windage::Algorithms::ChessboardDetector(width, height, size);

	return true;
}

bool MultipleMarkerTracking::UpdateCamerapose(IplImage* grayImage)
{
	if(initialize == false)
		return false;

	for(unsigned int i=0; i<this->detector.size(); i++)
	{
		if(this->detector[i]->FindMarker(grayImage))
		{
			estimator->AttatchCameraParameter(this->cameraParameter[i]);

			this->estimator->AttatchReferencePoint(this->detector[i]->GetReferencePoints());
			this->estimator->AttatchScenePoint(this->detector[i]->GetKeypoints());
			this->estimator->Calculate();

			this->refiner->AttatchHomography(estimator->GetHomography());
			this->refiner->AttatchReferencePoint(estimator->GetReferencePoint());
			this->refiner->AttatchScenePoint(estimator->GetScenePoint());
			this->refiner->Calculate();

			this->estimator->DecomposeHomography();
		}
	}

	return true;
}

void MultipleMarkerTracking::DrawOutLine(IplImage* colorImage, int objectID, bool drawCross)
{
	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(255, 255, 255);

	windage::Calibration* calibration = this->cameraParameter[objectID];
	int width = (int)this->chessParameter[objectID].x-1;
	int height = (int)this->chessParameter[objectID].y-1;
	double size = this->chessParameter[objectID].z;

	cvLine(colorImage, calibration->ConvertWorld2Image(-size, -size, 0.0),				calibration->ConvertWorld2Image(+width*size, -size, 0.0),			color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, -size, 0.0),		calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	calibration->ConvertWorld2Image(-size, +height*size, 0.0),			color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(-size, +height*size, 0.0),		calibration->ConvertWorld2Image(-size, -size, 0.0),					color2, 6);

	cvLine(colorImage, calibration->ConvertWorld2Image(-size, -size, 0.0),				calibration->ConvertWorld2Image(+width*size, -size, 0.0),			color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, -size, 0.0),		calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	calibration->ConvertWorld2Image(-size, +height*size, 0.0),			color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(-size, +height*size, 0.0),		calibration->ConvertWorld2Image(-size, -size, 0.0),					color, 2);

	if(drawCross)
	{
		cvLine(colorImage, calibration->ConvertWorld2Image(-size, -size, 0.0),			calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	color2, 6);
		cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, -size, 0.0),	calibration->ConvertWorld2Image(-size, +height*size, 0.0),			color2, 6);

		cvLine(colorImage, calibration->ConvertWorld2Image(-size, -size, 0.0),			calibration->ConvertWorld2Image(+width*size, +height*size, 0.0),	color, 2);
		cvLine(colorImage, calibration->ConvertWorld2Image(+width*size, -size, 0.0),	calibration->ConvertWorld2Image(-size, +height*size, 0.0),			color, 2);
	}
}

void MultipleMarkerTracking::DrawDebugInfo(IplImage* colorImage, int objectID)
{
	this->detector[objectID]->DrawMarkerInfo(colorImage);
}
