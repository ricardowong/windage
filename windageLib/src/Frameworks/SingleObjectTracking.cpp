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

#include "Algorithms/SIFTGPUdetector.h"
#include "Frameworks/SingleObjectTracking.h"
using namespace windage;
using namespace windage::Frameworks;

#include <highgui.h>
#include <windows.h>
#include <process.h>
IplImage* globalGrayImage = NULL;
CRITICAL_SECTION criticalSection;

unsigned int WINAPI FeatureDetectionThread(void* pArg)
{
	SingleObjectTracking* thisClass = (SingleObjectTracking*)pArg;
	windage::Algorithms::SIFTGPUdetector* detector = new windage::Algorithms::SIFTGPUdetector();

	while(thisClass->processThread)
	{
		if(thisClass->update)
		{
			detector->DoExtractKeypointsDescriptor(globalGrayImage);
			std::vector<windage::FeaturePoint>* sceneKeypoints = detector->GetKeypoints();

			for(unsigned int i=0; i<sceneKeypoints->size(); i++)
			{
				int count = 0;
				int index = thisClass->GetMatcher()->Matching((*sceneKeypoints)[i]);
				if(0 <= index && index < (int)thisClass->referenceRepository.size())
				{
					// if not tracked have point
					EnterCriticalSection(&criticalSection);
					{
						if(thisClass->referenceRepository[index].IsTracked() == false)
						{
							thisClass->referenceRepository[index].SetTracked(true);

							thisClass->refMatchedKeypoints.push_back(thisClass->referenceRepository[index]);
							thisClass->sceMatchedKeypoints.push_back((*sceneKeypoints)[i]);
						}
					}
					LeaveCriticalSection(&criticalSection);
					count++;
				}
			}

			thisClass->update = false;
		}
	}

	delete detector;

	return 0;
}

bool SingleObjectTracking::Initialize(int width, int height, bool printInfo)
{
	if(this->cameraParameter == NULL)
		return false;
	
	this->width = width;
	this->height = height;

	if(this->checker != NULL)
		this->checker->AttatchEstimator(this->estimator);
	
	if(prevImage) cvReleaseImage(&prevImage);
	prevImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	if( this->cameraParameter	== NULL ||
		this->matcher			== NULL ||
		this->estimator			== NULL)
	{
		this->initialize = false;
		return false;
	}

	if(printInfo)
	{
		std::cout << this->GetFunctionName() << " Initialize" << std::endl;
		if(this->matcher)
			std::cout << "\tFeature Matching : " << this->matcher->GetFunctionName() << std::endl;
		if(this->tracker)
			std::cout << "\tFeature Tracking : " << this->tracker->GetFunctionName() << std::endl;
		if(this->estimator)
			std::cout << "\tPose Estimation : " << this->estimator->GetFunctionName() << std::endl;
		if(this->filter)
			std::cout << "\tFilter : " << this->filter->GetFunctionName() << std::endl;
		std::cout << std::endl;
	}

	// create detection thread
	InitializeCriticalSection(&criticalSection);
	_beginthreadex(NULL, 0, FeatureDetectionThread, (void*)this, 0, NULL);

	this->estimator->AttatchCameraParameter(this->cameraParameter);
	this->initialize = true;
	return true;
}

bool SingleObjectTracking::TrainingReference(std::vector<windage::FeaturePoint>* referenceFeatures)
{
	if(this->initialize == false)
		return false;

	for(unsigned int i=0; i<referenceFeatures->size(); i++)
	{
		this->referenceRepository.push_back((*referenceFeatures)[i]);
	}

	this->matcher->Training(&this->referenceRepository);
	this->trained = true;
	return true;
}

bool SingleObjectTracking::UpdateCamerapose(IplImage* grayImage)
{
	if(initialize == false || trained == false)
		return false;

	// featur tracking routine
	EnterCriticalSection(&criticalSection);
	{
		std::vector<windage::FeaturePoint> sceneKeypoints;
		this->tracker->TrackFeatures(prevImage, grayImage, &sceMatchedKeypoints, &sceneKeypoints);

		int index = 0;
		for(unsigned int i=0; i<sceneKeypoints.size(); i++)
		{
			if(sceneKeypoints[i].IsOutlier() == false)
			{
				sceMatchedKeypoints[index] = sceneKeypoints[i];
				index++;
			}
			else // tracking fail feature
			{
				this->referenceRepository[refMatchedKeypoints[index].GetRepositoryID()].SetTracked(false);

				sceMatchedKeypoints.erase(sceMatchedKeypoints.begin() + index);
				refMatchedKeypoints.erase(refMatchedKeypoints.begin() + index);
			}
		}
	}
	LeaveCriticalSection(&criticalSection);

	if(this->step > this->detectionRatio || this->detectionRatio < 1) // detection routine (add new points)
	{
		step = 0;

		if(globalGrayImage) cvCopyImage(grayImage, globalGrayImage);
		else				globalGrayImage = cvCloneImage(grayImage);
		this->update = true;
	}

	if((int)refMatchedKeypoints.size() > MIN_FEATURE_POINTS_COUNT)
	{
		EnterCriticalSection(&criticalSection);
		{
			// pose estimate
			this->estimator->AttatchReferencePoint(&refMatchedKeypoints);
			this->estimator->AttatchScenePoint(&sceMatchedKeypoints);
			this->estimator->Calculate();

			// outlier checker
			if(checker)
			{
				this->checker->AttatchEstimator(this->estimator);
				this->checker->Calculate();

				int index = 0;
				for(int i=0; i<(int)refMatchedKeypoints.size(); i++)
				{
					if(refMatchedKeypoints[i].IsOutlier() == true)
					{
						this->referenceRepository[refMatchedKeypoints[i].GetRepositoryID()].SetTracked(false);

						refMatchedKeypoints.erase(refMatchedKeypoints.begin() + i);
						sceMatchedKeypoints.erase(sceMatchedKeypoints.begin() + i);
						i--;
					}
				}
			}

			// refinement
			if(refiner)
			{
				this->refiner->AttatchCalibration(this->estimator->GetCameraParameter());
				this->refiner->AttatchReferencePoint(&refMatchedKeypoints);
				this->refiner->AttatchScenePoint(&sceMatchedKeypoints);
				this->refiner->Calculate();
			}
		}
		LeaveCriticalSection(&criticalSection);

		// filtering
		if(filter)
		{
			windage::Vector3 T;
			T.x = this->cameraParameter->GetCameraPosition().val[0];
			T.y = this->cameraParameter->GetCameraPosition().val[1];
			T.z = this->cameraParameter->GetCameraPosition().val[2];

			for(int j=0; j<filterStep; j++)
			{
				filter->Predict();
				filter->Correct(T);
			}

			windage::Vector3 prediction = filter->Predict();
			filter->Correct(T);

			this->cameraParameter->SetCameraPosition(cvScalar(prediction.x, prediction.y, prediction.z));
		}
	}

	cvCopyImage(grayImage, this->prevImage);

	this->step++;
	return true;
}

void SingleObjectTracking::DrawOutLine(IplImage* colorImage, bool drawCross)
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

void SingleObjectTracking::DrawDebugInfo(IplImage* colorImage)
{
	int pointCount = (int)refMatchedKeypoints.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;

	EnterCriticalSection(&criticalSection);
	for(unsigned int i=0; i<refMatchedKeypoints.size(); i++)
	{
		CvPoint imagePoint = cvPoint((int)sceMatchedKeypoints[i].GetPoint().x, (int)sceMatchedKeypoints[i].GetPoint().y);

		cvCircle(colorImage, imagePoint, size+5, CV_RGB(0, 0, 0), CV_FILLED);
		cvCircle(colorImage, imagePoint, size, CV_RGB(255, 255, 0), CV_FILLED);
	}
	LeaveCriticalSection(&criticalSection);
}