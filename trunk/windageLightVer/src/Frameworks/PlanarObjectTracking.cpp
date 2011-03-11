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

#include "Frameworks/PlanarObjectTracking.h"
using namespace windage;
using namespace windage::Frameworks;

bool PlanarObjectTracking::Initialize(int width, int height, double realWidth, double realHeight, bool printInfo)
{
	if(this->cameraParameter == NULL)
		return false;
	
	this->width = width;
	this->height = height;
	this->realWidth = realWidth;
	this->realHeight = realHeight;

	if(this->checker != NULL)
		this->checker->AttatchEstimator(this->estimator);
	
	if(prevImage) cvReleaseImage(&prevImage);
	prevImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	if( this->cameraParameter	== NULL ||
		this->detector			== NULL ||
		this->matcher			== NULL ||
		this->estimator			== NULL)
	{
		this->initialize = false;
		return false;
	}

	if(printInfo)
	{
		std::cout << this->GetFunctionName() << " Initialize" << std::endl;
		if(this->detector)
			std::cout << "\tFeature Extractor : " << this->detector->GetFunctionName() << std::endl;
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

	this->estimator->AttatchCameraParameter(this->cameraParameter);
	this->initialize = true;
	return true;
}

bool PlanarObjectTracking::AttatchReferenceImage(IplImage* grayImage)
{
	if(grayImage == NULL)
		return false;

	if(this->referenceImage) cvReleaseImage(&this->referenceImage);
	this->referenceImage = NULL;
	this->referenceImage = cvCloneImage(grayImage);

	return true;
}

bool PlanarObjectTracking::TrainingReference(double scaleFactor, int scaleStep)
{
	if(this->referenceImage == NULL)
		return false;
	if(this->initialize == false)
		return false;

	int width = cvRound((double)this->referenceImage->width/scaleFactor);
	int height = cvRound((double)this->referenceImage->height/scaleFactor);
	int count = 0;

	this->referenceRepository.clear();
	for(int y=1; y<=scaleStep; y++)
	{
		for(int x=1; x<=scaleStep; x++)
		{
//			int x = y;
			IplImage* resizeReferenceImage = cvCreateImage(cvSize(width*x, height*y), IPL_DEPTH_8U, 1);
			cvResize(this->referenceImage, resizeReferenceImage, CV_INTER_LINEAR);
//			cvSmooth(resizeReferenceImage, resizeReferenceImage, CV_GAUSSIAN, 3, 3);

			this->detector->DoExtractKeypointsDescriptor(resizeReferenceImage);
			std::vector<windage::FeaturePoint>* tempReferenceKeypoints = this->detector->GetKeypoints();

			// add reference feature repository
			double xScaleFactor = this->realWidth / (double)resizeReferenceImage->width;
			double yScaleFactor = this->realHeight / (double)resizeReferenceImage->height;
			for(unsigned int i=0; i<tempReferenceKeypoints->size(); i++)
			{
				windage::Vector3 point = (*tempReferenceKeypoints)[i].GetPoint();
				point.x *= xScaleFactor;
				point.y *= yScaleFactor;
				point.x -= (double)this->realWidth/2.0;
				point.y = (double) this->realHeight/2.0 - point.y + 1;

				(*tempReferenceKeypoints)[i].SetPoint(point);
				(*tempReferenceKeypoints)[i].SetRepositoryID(count);

				this->referenceRepository.push_back((*tempReferenceKeypoints)[i]);
				count++;
			}

			cvReleaseImage(&resizeReferenceImage);
		}
	}

	this->matcher->Training(&this->referenceRepository);
	this->trained = true;
	return true;
}

bool PlanarObjectTracking::UpdateCamerapose(IplImage* grayImage)
{
	if(initialize == false || trained == false)
		return false;

	if(tracker == NULL)
		this->SetDitectionRatio(-1);
	
	if(this->detectionRatio < 0)
	{
		refMatchedKeypoints.clear();
		sceMatchedKeypoints.clear();
	}
	else // featur tracking routine
	{
		if(this->performance)
			this->performance->updateTickCount();

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

		if(this->performance)
			this->performance->log("tracking", this->performance->calculateProcessTime());
	}

	if(this->step > this->detectionRatio || this->detectionRatio < 1) // detection routine (add new points)
	{
		step = 0;

		if(this->performance)
			this->performance->updateTickCount();

		this->detector->DoExtractKeypointsDescriptor(grayImage);

		if(this->performance)
			this->performance->log("feature", this->performance->calculateProcessTime());

		std::vector<windage::FeaturePoint>* sceneKeypoints = this->detector->GetKeypoints();

		if(this->performance)
			this->performance->updateTickCount();

		for(unsigned int i=0; i<sceneKeypoints->size(); i++)
		{
			int count = 0;
			int index = this->matcher->Matching((*sceneKeypoints)[i]);
			if(0 <= index && index < (int)this->referenceRepository.size())
			{
				// if not tracked have point
				if(this->referenceRepository[index].IsTracked() == false)
				{
					if(this->detectionRatio >= 1)
						this->referenceRepository[index].SetTracked(true);

					refMatchedKeypoints.push_back(this->referenceRepository[index]);
					sceMatchedKeypoints.push_back((*sceneKeypoints)[i]);
				}
				count++;
			}
		}

		if(this->performance)
			this->performance->log("matching", this->performance->calculateProcessTime());
	}

	if(this->performance)
		this->performance->updateTickCount();

	int matchedCount = (int)refMatchedKeypoints.size();
	if(matchedCount > MIN_FEATURE_POINTS_COUNT)
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
			this->refiner->AttatchHomography(this->estimator->GetHomography());
			this->refiner->AttatchReferencePoint(&refMatchedKeypoints);
			this->refiner->AttatchScenePoint(&sceMatchedKeypoints);
			this->refiner->Calculate();
		}

		this->estimator->DecomposeHomography(this->cameraParameter);

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

	if(this->performance)
		this->performance->log("pose", this->performance->calculateProcessTime());


	if(this->performance)
	{
		this->performance->log("keypoint", (int)this->detector->GetKeypoints()->size());
		this->performance->log("match", matchedCount);
		this->performance->log("inlier", (int)refMatchedKeypoints.size());
		this->performance->logNewLine();
	}

	if(this->logger)
	{
		this->logger->log("keypoint", (int)this->detector->GetKeypoints()->size());
		this->logger->log("match", matchedCount);
		this->logger->log("inlier", (int)refMatchedKeypoints.size());
//		this->logger->logNewLine();
	}

	cvCopyImage(grayImage, this->prevImage);

	this->step++;
	return true;
}

void PlanarObjectTracking::DrawOutLine(IplImage* colorImage, bool drawCross)
{
	CvScalar color = CV_RGB(255, 0, 255);
	DrawOutLine(colorImage, color, drawCross);
}

void PlanarObjectTracking::DrawOutLine(IplImage* colorImage, CvScalar color, bool drawCross)
{
	int size = 4;

	CvScalar color2 = CV_RGB(255, 255, 255);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);

	if(drawCross)
	{
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);

		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);
	}
}

void PlanarObjectTracking::DrawDebugInfo(IplImage* colorImage)
{
	int pointCount = (int)refMatchedKeypoints.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;
	for(unsigned int i=0; i<refMatchedKeypoints.size(); i++)
	{
		double dx = refMatchedKeypoints[i].GetPoint().x * (double)colorImage->width/realWidth + (double)colorImage->width/2.0;
		double dy = (double)colorImage->height - refMatchedKeypoints[i].GetPoint().y * (double)colorImage->height/realHeight - (double)colorImage->height/2.0;
		CvPoint referencePoint = cvPoint(cvRound(dx), cvRound(dy));
		CvPoint imagePoint = cvPoint((int)sceMatchedKeypoints[i].GetPoint().x, (int)sceMatchedKeypoints[i].GetPoint().y);

		cvCircle(colorImage, referencePoint, size, CV_RGB(0, 255, 255), CV_FILLED);
		cvCircle(colorImage, imagePoint, size, CV_RGB(255, 255, 0), CV_FILLED);

		cvLine(colorImage, referencePoint, imagePoint, CV_RGB(255, 0, 0));
	}
}