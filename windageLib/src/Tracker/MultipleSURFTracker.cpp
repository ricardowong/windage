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

#include <omp.h>

#include "Tracker/MultipleSURFTracker.h"
#include "Tracker/PoseEstimation/FindPROSACHomography.h"
#include "Tracker/PoseEstimation/FindEpnpPoseEstimation.h"
#include "Tracker/PoseEstimation/Find3DPoseEstimation.h"

//#define POSE_3D_ESTIMATION

const double ERROR_BOUND = 3.0;
const int POSE_POINTS_COUNT = 10;
const double COMPAIR_RATE = 0.50;
const int EMAX = 20;

const double PROSAC_TIME_OUT = 5.0;

using namespace windage;
void MultipleSURFTracker::Release()
{
	for(int i=0; i<(int)referenceStorageList.size(); i++)
	{
		delete referenceStorageList[i];
	}
	referenceStorageList.clear();

	if(prevImage) cvReleaseImage(&prevImage);
}

void MultipleSURFTracker::AttatchReferenceImage(IplImage* image, double realWidth, double realHeight, double scaleFactor, int scaleStep)
{
	IplImage* temp = cvCloneImage(image);
	referenceImageList.push_back(temp);

	double* params = GetCameraParameter()->GetParameters();
	windage::Calibration* tempCalibration = new windage::Calibration();
	tempCalibration->Initialize(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7]);
	cameraParameterList.push_back(tempCalibration);

	SURFReferenceStorage* referenceStorage = new SURFReferenceStorage();
	referenceStorage->SetFeatureExtractTreshold(this->featureExtractThreshold);
	referenceStorage->RegistReferenceImage(image, realWidth, realHeight, scaleFactor, scaleStep);
	referenceStorageList.push_back(referenceStorage);

	referenceCount = referenceStorageList.size();
	this->sceneSURF.resize(referenceCount);
	this->matchedReferenceIndex.resize(referenceCount);
}

bool MultipleSURFTracker::DeleteReferenceImage(int index)
{
	if(index < (int)referenceImageList.size())
	{
		if(referenceImageList[index]) cvReleaseImage(&referenceImageList[index]);
		referenceImageList.erase(referenceImageList.begin()+index);

		if(cameraParameterList[index]) delete cameraParameterList[index];
		cameraParameterList.erase(cameraParameterList.begin()+index);

		if(referenceStorageList[index]) delete referenceStorageList[index];
		referenceStorageList.erase(referenceStorageList.begin()+index);

		referenceCount = referenceStorageList.size();
		this->sceneSURF.resize(referenceCount);
		this->matchedReferenceIndex.resize(referenceCount);
		return true;
	}
	else
	{
		return false;
	}
}

double MultipleSURFTracker::CalculatePose(int index)
{
	float homography[9];
	float intrinsicMatrix[9];
	float extrinsicMatrix[12];
	float extrinsicOutMatrix[16];

	CvMat* intrinsic = cameraParameter->GetIntrinsicMatrix();
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			intrinsicMatrix[y*3+x] = (float)cvGetReal2D(intrinsic, y, x);
		}
	}

	// find pair set
	std::vector<CvPoint2D32f> matchedReferencePoints;
	std::vector<CvPoint2D32f> matchedScenePoints;
	std::vector<float> distanceList;

	SURFReferenceStorage* reference = referenceStorageList[index];
	std::vector<SURFDesciription>* descriptor = reference->GetDescriptor();
	for(int j=0; j<(int)sceneSURF[index].size(); j++)
	{
		matchedReferencePoints.push_back((*descriptor)[matchedReferenceIndex[index][j]].point);
		matchedScenePoints.push_back(sceneSURF[index][j].point);
		distanceList.push_back(sceneSURF[index][j].distance);
	}

	// calculate object pose
	float homographyError = 0;
	int n = (int)matchedReferencePoints.size();
	if(n >= POSE_POINTS_COUNT)
	{
		int inlierCount = 0;
		int outlierCount = 0;

		CvMat _h = cvMat(3, 3, CV_32F, homography);

		CvMat _intrinsic = cvMat(3, 3, CV_32F, intrinsicMatrix);
		CvMat _extrinsic = cvMat(3, 4, CV_32FC1, extrinsicMatrix);

		homographyError = 0.0;

		CvMat _pt1 = cvMat(1, n, CV_32FC2, &(matchedReferencePoints[0]) );
		CvMat _pt2 = cvMat(1, n, CV_32FC2, &(matchedScenePoints[0]) );

		bool isCalculate = false;
		switch(this->poseEstimationMethod)
		{
		case windage::PROSAC:
			{
				FindPROSACHomography prosac;
				prosac.SetTimeout(PROSAC_TIME_OUT);

				std::vector<MatchedPoint> prosacMatchedPoints;
				for(int i=0; i<(int)matchedScenePoints.size(); i++)
				{
					MatchedPoint referenceTemp(matchedScenePoints[i], matchedReferencePoints[i], distanceList[i]);
					prosacMatchedPoints.push_back(referenceTemp);
				}
				prosac.SetReprojectionThreshold((float)ERROR_BOUND);
				prosac.AttatchMatchedPoints(&prosacMatchedPoints);

				isCalculate = prosac.Calculate();
				if(isCalculate)
				{
					float* tempHomography = prosac.GetHomography();
					for(int i=0; i<9; i++) homography[i] = tempHomography[i];
				}
			}
			break;
		case windage::RANSAC:
			{				
				if(cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, ERROR_BOUND) > 0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		case windage::LMEDS:
			{
				if(cvFindHomography( &_pt1, &_pt2, &_h, CV_LMEDS) > 0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		case windage::POSE_3D:
			{
				Find3DPoseEstimation poseEstimator;
				std::vector<Matched3DPoint> poseEstimatorPoints;
				for(int i=0; i<(int)matchedScenePoints.size(); i++)
				{
					Matched3DPoint referenceTemp(matchedScenePoints[i], cvPoint3D32f(matchedReferencePoints[i].x, matchedReferencePoints[i].y, 0.0));
					poseEstimatorPoints.push_back(referenceTemp);
				}
				poseEstimator.AttatchMatchedPoints(&poseEstimatorPoints);
				poseEstimator.AttatchCalibration(this->cameraParameterList[index]);
				poseEstimator.SetReprojectionThreshold(ERROR_BOUND);
				if(poseEstimator.Calculate() > 0.0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		}

		// outlier remove
		if(outlinerRemove && isCalculate)
		{
			// calculate homography error & remove outlier
			float difference = 0;
			int count = 0;
			for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
			{
				float error = 0.0;
				if(this->poseEstimationMethod == windage::POSE_3D)
				{
					error = Find3DPoseEstimation::ComputeReprojError(matchedScenePoints[i], cvPoint3D32f(matchedReferencePoints[i].x, matchedReferencePoints[i].y, 0.0),
						cameraParameter->GetIntrinsicMatrix(), cameraParameterList[index]->GetExtrinsicMatrix());
				}
				else
				{
					error = FindHomography::ComputeReprojError(matchedReferencePoints[i], matchedScenePoints[i], homography);
				}

				if(error <= ERROR_BOUND)
				{
					distanceList[count] /= 10.0;
					distanceList[count] /= 10.0;

					count++;
					inlierCount++;
				}
				else // outlier
				{
					this->matchedReferenceIndex[index].erase(matchedReferenceIndex[index].begin() + count);
					this->sceneSURF[index].erase(sceneSURF[index].begin() + count);
					outlierCount++;
				}

				difference += (error);
			}
			homographyError = difference / (float)matchedReferencePoints.size();
		}

		if(this->poseEstimationMethod == windage::POSE_3D)
		{
			if(isCalculate)
			{
			}
			else
			{
				// remove all points
				if(outlinerRemove)
				{
					this->matchedReferenceIndex[index].clear();
					this->sceneSURF[index].clear();
				}

				homographyError = -1.0;
			}
		}
		else
		{
			if(isCalculate)
			{
#ifndef POSE_3D_ESTIMATION
				DecomposeHomographyToRT(&_intrinsic, &_h, &_extrinsic);
				for(int y=0; y<3; y++)
				{
					for(int x=0; x<4; x++)
					{
						extrinsicOutMatrix[y*4 + x] = extrinsicMatrix[y*4 + x];
					}
				}
				extrinsicOutMatrix[3*4 + 0] = extrinsicOutMatrix[3*4 + 1] = extrinsicOutMatrix[3*4 + 2] = 0;
				extrinsicOutMatrix[3*4 + 3] = 1;

				this->cameraParameterList[index]->SetExtrinsicMatrix(extrinsicOutMatrix);
#else
				Find3DPoseEstimation poseEstimator;
				std::vector<Matched3DPoint> poseEstimatorPoints;
				for(int i=0; i<(int)matchedScenePoints.size(); i++)
				{
					Matched3DPoint referenceTemp(matchedScenePoints[i], cvPoint3D32f(matchedReferencePoints[i].x, matchedReferencePoints[i].y, 0.0));
					poseEstimatorPoints.push_back(referenceTemp);
				}
				poseEstimator.AttatchMatchedPoints(&poseEstimatorPoints);
				poseEstimator.AttatchCalibration(this->cameraParameterList[index]);
				poseEstimator.SetReprojectionThreshold(ERROR_BOUND);
//				poseEstimator.SetUseRANSAC(false);
				poseEstimator.Calculate();
#endif
			}
			else
			{
				// remove all points
				if(outlinerRemove)
				{
					this->matchedReferenceIndex[index].clear();
					this->sceneSURF[index].clear();
				}

				homographyError = -1.0;
			}
		}
	}
	else
	{
		// remove all points
		if(outlinerRemove && n < POSE_POINTS_COUNT/2)
		{
			this->matchedReferenceIndex[index].clear();
			this->sceneSURF[index].clear();
		}

		homographyError = -1.0;
	}	

	return homographyError;
}

int MultipleSURFTracker::DetectObject(std::vector<SURFDesciription>* scene, std::vector<int>* matchedIndex, int index)
{
	int count = 0;
	SURFReferenceStorage* reference = referenceStorageList[index];
	for(int i=0; i<(int)(*scene).size(); i++)
	{
		float distance = 0;
		int featureIndex = reference->FindPairs((*scene)[i], COMPAIR_RATE, EMAX, &distance);
		if(featureIndex > 0)
		{
			(*scene)[i].objectID = index;
			(*scene)[i].distance = distance;
			(*matchedIndex)[i] = featureIndex;
			count++;
		}

	}
	return count;
}

double MultipleSURFTracker::UpdateCameraPose(IplImage* grayImage)
{
	// opticalflow sequence
	{
		std::vector<CvPoint2D32f> matchedScenePoints;
		std::vector<CvPoint2D32f> matchedTempPoints;
		for(int i=0; i<(int)sceneSURF.size(); i++)
		{
			for(int j=0; j<(int)sceneSURF[i].size(); j++)
			{
				matchedScenePoints.push_back(sceneSURF[i][j].point);
			}
		}

		opticalflow->TrackFeature(prevImage, grayImage, &matchedScenePoints, &matchedTempPoints);

		int trackedCount = 0;
		for(int i=0; i<(int)sceneSURF.size(); i++)
		{
			int index = 0;
			for(int j=0; j<(int)sceneSURF[i].size(); j++)
			{
				if((int)matchedTempPoints.size() > trackedCount)
				{
					if(matchedTempPoints[trackedCount].x >= 0 && matchedTempPoints[trackedCount].y >= 0)
					{
						sceneSURF[i][index].point = matchedTempPoints[trackedCount];
						index++;
					}
					else
					{
						sceneSURF[i].erase(sceneSURF[i].begin() + index);
						matchedReferenceIndex[i].erase(matchedReferenceIndex[i].begin() + index);
					}
					trackedCount++;
				}
			}
		}
	}

	// detection sequence
	{
		// selected object detection
		int start = 0;
		int end = 0;
		if(interval >= 1) 
		{
			int round = cvRound(interval);
			if(step%round == 0)
			{
				start = step/round;
				end = MIN(start+1, referenceCount);
			}
		}
		else if(interval == 0)
		{
		}
		else 
		{
			int round = cvRound(1 / interval);
			start = step * round;
			end = MIN(start+round, referenceCount);
		}

		for(int i=start; i<end; i++)
		{
			std::vector<CvPoint> fastCorners;
			std::vector<SURFDesciription> tempSceneSURF;
			this->featureCount = ModifiedSURFTracker::ExtractFASTCorner(&fastCorners, grayImage, featureExtractThreshold);
			ModifiedSURFTracker::ExtractModifiedSURF(grayImage, &fastCorners, &tempSceneSURF);

			std::vector<int> matchedIndex;
			matchedIndex.resize(tempSceneSURF.size());

			
			int matchedCount = DetectObject(&tempSceneSURF, &matchedIndex, i);

			// attatch scene points
			for(int t=0; t<(int)tempSceneSURF.size(); t++)
			{
				int objectID = tempSceneSURF[t].objectID;
				if(objectID >= 0)
				{
					bool isFound = false;
					for(int j=0; j<(int)sceneSURF[objectID].size()&&!isFound; j++)
					{
						if(abs(sceneSURF[objectID][j].point.x - tempSceneSURF[t].point.x) + abs(sceneSURF[objectID][j].point.y - tempSceneSURF[t].point.y) <= 2.0f)
							isFound = true;
					}

					if(!isFound)
					{
						sceneSURF[objectID].push_back(tempSceneSURF[t]);
						matchedReferenceIndex[objectID].push_back(matchedIndex[t]);
					}
				}
			}
		}
		step++;
		if(step >= referenceCount*interval) step = 0;
	}

	// update object pose
	for(int i=0; i<referenceCount; i++)
	{
		double error = CalculatePose(i);
	}

	if(prevImage)	cvCopyImage(grayImage, prevImage);
	else			prevImage = cvCloneImage(grayImage);

	return 0;
}

void MultipleSURFTracker::DrawDebugInfo(IplImage* colorImage)
{
	const int size = 5;
	const int lineThick = 1;
	double count = this->referenceCount-1;
	for(int i=0; i<(int)sceneSURF.size(); i++)
	{
		double ratio = (double)i/count;
		double increase = ratio * 255.0;
		double decrease = (1-ratio) * 255.0;

		CvScalar imageColor = CV_RGB(increase, 255.0, decrease);
		CvScalar referColor = CV_RGB(decrease, 255.0, increase);
		CvScalar lineColor = CV_RGB(decrease, 0.0, increase);

		Calibration* calibration = this->cameraParameterList[i];
		double realWidth = this->referenceStorageList[i]->GetRealWidth();
		double realHeight = this->referenceStorageList[i]->GetRealHeight();
		std::vector<SURFDesciription>* descriptor = this->referenceStorageList[i]->GetDescriptor();

		for(int j=0; j<(int)sceneSURF[i].size(); j++)
		{
			CvPoint2D32f referPoint = (*descriptor)[this->matchedReferenceIndex[i][j]].point;
			CvPoint referencePoint = cvPoint((int)referPoint.x * colorImage->width/realWidth + colorImage->width/2,
									(int)(colorImage->height - referPoint.y * colorImage->height/realHeight - colorImage->height/2));;
			CvPoint imagePoint = cvPoint((int)sceneSURF[i][j].point.x, (int)sceneSURF[i][j].point.y);

			cvCircle(colorImage, referencePoint, size, referColor, CV_FILLED);
			cvCircle(colorImage, imagePoint, size, imageColor, CV_FILLED);

			cvLine(colorImage, referencePoint, imagePoint, lineColor, lineThick);
		}
	}
}

void MultipleSURFTracker::DrawDebugInfo2(IplImage* colorImage, int index)
{
	const int size = 10;
	const int lineThick = 3;
	double count = this->referenceCount-1;
	double ratio = (double)index/count;
	double increase = 0.0;//ratio * 255.0;
	double decrease = 255.0;//(1-ratio) * 255.0;

	CvScalar imageColor = CV_RGB(increase, 255.0, decrease);
	CvScalar referColor = CV_RGB(decrease, 255.0, increase);
	CvScalar lineColor = CV_RGB(decrease, 0.0, increase);

	Calibration* calibration = this->cameraParameterList[index];
	double realWidth = this->referenceStorageList[index]->GetRealWidth();
	double realHeight = this->referenceStorageList[index]->GetRealHeight();
	std::vector<SURFDesciription>* descriptor = this->referenceStorageList[index]->GetDescriptor();

	for(int j=0; j<(int)sceneSURF[index].size(); j++)
	{
		CvPoint2D32f referPoint = (*descriptor)[this->matchedReferenceIndex[index][j]].point;
		CvPoint referencePoint = cvPoint((int)referPoint.x * colorImage->width/realWidth + colorImage->width/2,
								(int)((colorImage->height/2) - referPoint.y * (colorImage->height/2)/realHeight - colorImage->height/4));
		CvPoint imagePoint = cvPoint((int)sceneSURF[index][j].point.x, (int)sceneSURF[index][j].point.y + colorImage->height/2);

		cvCircle(colorImage, referencePoint, size, referColor, CV_FILLED);
		cvCircle(colorImage, imagePoint, size, imageColor, CV_FILLED);

		cvLine(colorImage, referencePoint, imagePoint, lineColor, lineThick);
	}
}

void MultipleSURFTracker::DrawOutLine(IplImage* colorImage, int index, bool drawCross)
{
	int pointCount = (int)sceneSURF.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;

	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(255, 255, 255);

	Calibration* calibration = this->cameraParameterList[index];
	double realWidth = this->referenceStorageList[index]->GetRealWidth()/2;
	double realHeight = this->referenceStorageList[index]->GetRealHeight()/2;

	cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	color2, 6);
	cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	color2, 6);

	cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	color, 2);
	cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	color, 2);

	if(drawCross)
	{
		cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	color2, 6);
		cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	color2, 6);

		cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, -realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, +realHeight, 0.0),	color, 2);
		cvLine(colorImage, calibration->ConvertWorld2Image(-realWidth, +realHeight, 0.0),	calibration->ConvertWorld2Image(+realWidth, -realHeight, 0.0),	color, 2);
	}
}

void MultipleSURFTracker::DrawInfomation(IplImage* colorImage, int index, double size)
{
	cameraParameterList[index]->DrawInfomation(colorImage, size);
}