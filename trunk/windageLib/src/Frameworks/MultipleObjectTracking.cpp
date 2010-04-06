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
#include "Frameworks/MultipleObjectTracking.h"
using namespace windage;
using namespace windage::Frameworks;

#include <windows.h>
#include <process.h>
namespace MultipleOjbectThread
{
	IplImage* globalGrayImage = NULL;
	IplImage* globalCurrentGrayImage = NULL;
	CRITICAL_SECTION csKeypointsUpdate;
	CRITICAL_SECTION csImageUpdate;

	unsigned int WINAPI FeatureDetectionThread(void* pArg)
	{
		windage::Frameworks::MultipleObjectTracking* thisClass = (windage::Frameworks::MultipleObjectTracking*)pArg;
		windage::Algorithms::SIFTGPUdetector* detector = new windage::Algorithms::SIFTGPUdetector();
		windage::Algorithms::OpticalFlow* tracker = new windage::Algorithms::OpticalFlow(50);
		tracker->Initialize(thisClass->GetSize().width, thisClass->GetSize().height, cvSize(15, 15), 3);

		cvGetTickCount();

		while(thisClass->processThread)
		{
			if(thisClass->update)
			{
				int objectID = thisClass->objectID;
				std::vector<windage::FeaturePoint> refMatchedKeypoints;
				std::vector<windage::FeaturePoint> sceMatchedKeypoints;

				// detect feature
				detector->DoExtractKeypointsDescriptor(globalGrayImage);
				std::vector<windage::FeaturePoint>* sceneKeypoints = detector->GetKeypoints();

				for(unsigned int i=0; i<sceneKeypoints->size(); i++)
				{
					int count = 0;
					int index = thisClass->GetMatcher(objectID)->Matching((*sceneKeypoints)[i]);
					if(0 <= index && index < (int)thisClass->referenceRepository[objectID].size())
					{
						(*sceneKeypoints)[i].SetRepositoryID(index);

						refMatchedKeypoints.push_back(thisClass->referenceRepository[objectID][index]);
						sceMatchedKeypoints.push_back((*sceneKeypoints)[i]);
					}
				}

				if(sceMatchedKeypoints.size() > 1)
				{
					// track feature
					std::vector<windage::FeaturePoint> sceneUpdatedKeypoints;
					EnterCriticalSection(&csImageUpdate);
					tracker->TrackFeatures(globalGrayImage, globalCurrentGrayImage, &sceMatchedKeypoints, &sceneUpdatedKeypoints);
					LeaveCriticalSection(&csImageUpdate);

					for(unsigned int i=0; i<sceneUpdatedKeypoints.size(); i++)
					{
						// if not tracked have point
						int index = sceneUpdatedKeypoints[i].GetRepositoryID();
						if(sceneUpdatedKeypoints[i].IsOutlier() == false && thisClass->referenceRepository[objectID][index].IsTracked() == false)
						{
							EnterCriticalSection(&csKeypointsUpdate);
							{
								thisClass->referenceRepository[objectID][index].SetTracked(true);

								thisClass->refMatchedKeypoints[objectID].push_back(thisClass->referenceRepository[objectID][index]);
								thisClass->sceMatchedKeypoints[objectID].push_back((sceneUpdatedKeypoints)[i]);
							}
							LeaveCriticalSection(&csKeypointsUpdate);
						}
					}
				}

				thisClass->update = false;
			}
		}

		delete detector;
		delete tracker;

		return 0;
	}
};

bool MultipleObjectTracking::Initialize(int width, int height, bool printInfo)
{
	if(this->initialCamearParameter == NULL)
		return false;
	
	this->width = width;
	this->height = height;
	
	if(prevImage) cvReleaseImage(&prevImage);
	prevImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	if( this->initialCamearParameter	== NULL ||
		this->tracker					== NULL ||
		this->estimator					== NULL)
	{
		this->initialize = false;
		return false;
	}

	if(printInfo)
	{
		std::cout << this->GetFunctionName() << " Initialize" << std::endl;
		if(this->detector)
			std::cout << "\tFeature Extractor : " << this->detector->GetFunctionName() << std::endl;
		if(this->tracker)
			std::cout << "\tFeature Tracking : " << this->tracker->GetFunctionName() << std::endl;
		if(this->estimator)
			std::cout << "\tPose Estimation : " << this->estimator->GetFunctionName() << std::endl;
		std::cout << std::endl;
	}

	// create detection thread
	InitializeCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);
	InitializeCriticalSection(&MultipleOjbectThread::csImageUpdate);
	_beginthreadex(NULL, 0, MultipleOjbectThread::FeatureDetectionThread, (void*)this, 0, NULL);

	this->initialize = true;
	return true;
}

bool MultipleObjectTracking::TrainingReference(std::vector<windage::FeaturePoint>* referenceFeatures)
{
	if(this->initialize == false)
		return false;

	this->referenceRepository.resize(this->objectCount+1);
	this->refMatchedKeypoints.resize(this->objectCount+1);
	this->sceMatchedKeypoints.resize(this->objectCount+1);
	this->searchTree.resize(this->objectCount+1);

	for(unsigned int i=0; i<referenceFeatures->size(); i++)
	{
		(*referenceFeatures)[i].SetObjectID(this->objectCount);
		(*referenceFeatures)[i].SetRepositoryID(i);
		this->referenceRepository[this->objectCount].push_back((*referenceFeatures)[i]);
	}
	this->searchTree[this->objectCount] = new SearchTreeT();
	this->searchTree[this->objectCount]->Training(&this->referenceRepository[this->objectCount]);
	this->searchTree[this->objectCount]->SetRatio(SEARCH_TREE_RATIO);

	this->cameraParameter.resize(this->objectCount+1);
	this->cameraParameter[this->objectCount] = new windage::Calibration();
	this->cameraParameter[this->objectCount]->Initialize(	this->initialCamearParameter->GetParameters()[0],
															this->initialCamearParameter->GetParameters()[1],
															this->initialCamearParameter->GetParameters()[2],
															this->initialCamearParameter->GetParameters()[3],
															this->initialCamearParameter->GetParameters()[4],
															this->initialCamearParameter->GetParameters()[5],
															this->initialCamearParameter->GetParameters()[6],
															this->initialCamearParameter->GetParameters()[7]);

	this->estimatorList.resize(this->objectCount+1);
	this->estimatorList[this->objectCount] = new PoseEstimationT();
	this->estimatorList[this->objectCount]->SetReprojectionError(this->estimator->GetReprojectionError());
	this->estimatorList[this->objectCount]->SetConfidence(((PoseEstimationT*)this->estimator)->GetConfidence());
	this->estimatorList[this->objectCount]->SetMaxIteration(((PoseEstimationT*)this->estimator)->GetMaxIteration());

	this->objectCount++;
	this->detectionStep = this->objectCount * this->detectionRatio;
	this->trained = true;

	return true;
}

bool MultipleObjectTracking::UpdateCamerapose(IplImage* grayImage)
{
	if(initialize == false || trained == false)
		return false;

	// featur tracking routine
	std::vector<windage::FeaturePoint> sceneKeypoints1;
	std::vector<windage::FeaturePoint> sceneKeypoints2;

	EnterCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);
	{
		for(unsigned int i=0; i<this->sceMatchedKeypoints.size(); i++)
		{
			for(unsigned int j=0; j<this->sceMatchedKeypoints[i].size(); j++)
			{
				sceneKeypoints1.push_back(this->sceMatchedKeypoints[i][j]);
			}
		}

		this->tracker->TrackFeatures(prevImage, grayImage, &sceneKeypoints1, &sceneKeypoints2);

		int iter = 0;
		for(int i=0; i<this->objectCount; i++)
		{
			int index = 0;
			for(unsigned int j=0; j<this->sceMatchedKeypoints[i].size(); j++)
			{
				if(sceneKeypoints2[iter].IsOutlier() == false)
				{
					sceMatchedKeypoints[i][index] = sceneKeypoints2[iter];
					index++;
				}
				else 
				{
					this->referenceRepository[i][refMatchedKeypoints[i][index].GetRepositoryID()].SetTracked(false);

					sceMatchedKeypoints[i].erase(sceMatchedKeypoints[i].begin() + index);
					refMatchedKeypoints[i].erase(refMatchedKeypoints[i].begin() + index);
					j--;
				}
				iter++;
			}
			
		}
	}
	LeaveCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);

	// feature detection
	{
		EnterCriticalSection(&MultipleOjbectThread::csImageUpdate);
		if(MultipleOjbectThread::globalCurrentGrayImage)	cvCopyImage(grayImage, MultipleOjbectThread::globalCurrentGrayImage);
		else						MultipleOjbectThread::globalCurrentGrayImage = cvCloneImage(grayImage);
		LeaveCriticalSection(&MultipleOjbectThread::csImageUpdate);

		int objectID = this->step;
		if(objectID % this->detectionRatio == 0)
		{
			objectID /= this->detectionRatio;
			if(objectID < this->objectCount)
			{
				if(MultipleOjbectThread::globalGrayImage) cvCopyImage(grayImage, MultipleOjbectThread::globalGrayImage);
				else				MultipleOjbectThread::globalGrayImage = cvCloneImage(grayImage);
				this->objectID = objectID;
				this->update = true;
			}
		}
	}

	// pose estimate
	EnterCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);

	#pragma omp parallel for
	for(int i=0; i<this->objectCount; i++)
	{
		if((int)refMatchedKeypoints[i].size() > MIN_FEATURE_POINTS_COUNT)
		{
/*
			this->estimator->AttatchCameraParameter(this->cameraParameter[i]);
			this->estimator->AttatchReferencePoint(&(refMatchedKeypoints[i]));
			this->estimator->AttatchScenePoint(&(sceMatchedKeypoints[i]));
			this->estimator->Calculate();
//*/
			this->estimatorList[i]->AttatchCameraParameter(this->cameraParameter[i]);
			this->estimatorList[i]->AttatchReferencePoint(&(refMatchedKeypoints[i]));
			this->estimatorList[i]->AttatchScenePoint(&(sceMatchedKeypoints[i]));
			this->estimatorList[i]->Calculate();

			// outlier rejection
			for(int j=0; j<(int)refMatchedKeypoints[i].size(); j++)
			{
				if(refMatchedKeypoints[i][j].IsOutlier() == true)
				{
					this->referenceRepository[i][refMatchedKeypoints[i][j].GetRepositoryID()].SetTracked(false);

					refMatchedKeypoints[i].erase(refMatchedKeypoints[i].begin() + j);
					sceMatchedKeypoints[i].erase(sceMatchedKeypoints[i].begin() + j);
					j--;
				}
			}

			// refinement
			if(this->refiner && sceMatchedKeypoints[i].size() > MIN_FEATURE_POINTS_COUNT)
			{
				#pragma omp critical
				{
					this->refiner->AttatchCalibration(this->cameraParameter[i]);
					this->refiner->AttatchReferencePoint(&(refMatchedKeypoints[i]));
					this->refiner->AttatchScenePoint(&(sceMatchedKeypoints[i]));
					this->refiner->Calculate();
				}
			}

		}
	}
	
	LeaveCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);

	cvCopyImage(grayImage, this->prevImage);
	this->step++;
	if(this->step >= this->detectionStep)
		this->step = 0;
	return true;
}

void MultipleObjectTracking::DrawDebugInfo(IplImage* colorImage, int objectID)
{
	int pointCount = (int)refMatchedKeypoints.size();
	int r = 255;
	int g = 0;
	int b = 0;

	double count = (double)this->objectCount - 1;
	
	EnterCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);
	int j = objectID;
	if(count > 0)
	{
		r = cvRound((double)(count - j)/count * 255.0);
		g = cvRound((double)(count - j)/count * 255.0);
		b = cvRound((double)j/count * 255.0);
	}
	else
	{
		r = cvRound((double)255.0);
		g = cvRound((double)255.0);
		b = cvRound((double)255.0);
	}

//	int size = 3;
	for(unsigned int i=0; i<refMatchedKeypoints[j].size(); i++)
	{
		CvPoint imagePoint = cvPoint((int)sceMatchedKeypoints[j][i].GetPoint().x, (int)sceMatchedKeypoints[j][i].GetPoint().y);
		int size = sceMatchedKeypoints[j][i].GetSize();

		cvCircle(colorImage, imagePoint, size+2, CV_RGB(0, 0, 0), CV_FILLED);
		cvCircle(colorImage, imagePoint, size, CV_RGB(r, g, b), CV_FILLED);
	}
	LeaveCriticalSection(&MultipleOjbectThread::csKeypointsUpdate);
}