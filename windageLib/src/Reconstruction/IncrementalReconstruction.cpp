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

#include <iostream>

#include "Reconstruction/BundleWrapper.h"
#include "Algorithms/EPnPRANSACestimator.h"

bool IncrementalReconstruction::Matching(std::vector<windage::FeaturePoint>* feature1, std::vector<windage::FeaturePoint>* feature2, std::vector<windage::FeaturePoint>* matchedPoint1, std::vector<windage::FeaturePoint>* matchedPoint2)
{
	searchtree->Training(feature1);
	for(unsigned int i=0; i<feature2->size(); i++)
	{
		int index = searchtree->Matching((*feature2)[i]);
		if(index >= 0)
		{
			(*feature1)[index].SetRepositoryID(index);
			(*feature2)[i].SetRepositoryID(i);

			matchedPoint1->push_back((*feature1)[index]);
			matchedPoint2->push_back((*feature2)[i]);
		}
	}
	return true;
}

int IncrementalReconstruction::MatchingCount(std::vector<windage::FeaturePoint>* feature1, std::vector<windage::FeaturePoint>* feature2)
{
	int count = 0;

	searchtree->Training(feature1);
	for(unsigned int i=0; i<feature2->size(); i++)
	{
		int index = searchtree->Matching((*feature2)[i]);
		if(index >= 0)
		{
			count++;
		}
	}

	return count;
}

bool IncrementalReconstruction::StereoReconstruction(int index1, int index2)
{
	if(this->attatchedCount < 2)
		return false;

	// release before data
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		reconstructionPoints[i].clear();
	}
	
	// matching
	std::vector<windage::FeaturePoint>* feature1 = &featurePointsList[index1];
	std::vector<windage::FeaturePoint>* feature2 = &featurePointsList[index2];

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	windage::Reconstruction::StereoReconstruction stereo;
	stereo.AttatchBaseCameraParameter(this->GetCameraParameter(index1));
	stereo.AttatchUpdateCameraParameter(this->GetCameraParameter(index2));

	stereo.AttatchMatchedPoint1(&matchedPoint1);
	stereo.AttatchMatchedPoint2(&matchedPoint2);

	double error = 0.0;
	stereo.CalculateNormalizedPoint();
	stereo.ComputeEssentialMatrixRANSAC(&error);

	std::vector<windage::ReconstructionPoint>* pose3D = stereo.GetReconstructionPoints();
	int count = 0;
	for(unsigned int i=0; i<pose3D->size(); i++)
	{
		if((*pose3D)[i].IsOutlier() == false)
		{
			windage::ReconstructionPoint reconsturctionPoint1;
			windage::ReconstructionPoint reconsturctionPoint2;

			reconsturctionPoint1.SetPoint((*pose3D)[i].GetPoint());
			reconsturctionPoint2.SetPoint((*pose3D)[i].GetPoint());
			reconsturctionPoint1.SetOutlier(false);
			reconsturctionPoint2.SetOutlier(false);

			reconsturctionPoint1.SetObjectID(index1);
			reconsturctionPoint2.SetObjectID(index2);
			reconsturctionPoint1.SetFeatureID(matchedPoint1[i].GetRepositoryID());
			reconsturctionPoint2.SetFeatureID(matchedPoint2[i].GetRepositoryID());
			reconsturctionPoint1.SetColor(matchedPoint1[i].GetColor());
			reconsturctionPoint2.SetColor(matchedPoint2[i].GetColor());
			reconsturctionPoint1.SetImagePoint(matchedPoint1[i].GetPoint());
			reconsturctionPoint2.SetImagePoint(matchedPoint2[i].GetPoint());

			// reconstructed point is tracked
			this->featurePointsList[index1][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[index2][matchedPoint2[i].GetRepositoryID()].SetTracked(true);

			this->reconstructionPoints[index1].push_back(reconsturctionPoint1);
			this->reconstructionPoints[index2].push_back(reconsturctionPoint2);
		}
	}

	std::cout << "stereo reconstruction : " << stereo.GetInlierCount() << " / " << matchedPoint2.size() << std::endl;

	this->caculatedCount = 2;
	return true;
}

bool IncrementalReconstruction::IncrementReconstruction(std::vector<windage::FeaturePoint>* feature)
{
	if(this->caculatedCount < 2)
		return false;

	// find max matching scene
	// temporary index;
	int index = this->caculatedCount - 1;

	// matching
	std::vector<windage::FeaturePoint> feature1;
	for(unsigned int i=0; i<this->reconstructionPoints[index].size(); i++)
	{
		int featureID = this->reconstructionPoints[index][i].GetFeatureID();
		windage::Vector4 point3D = this->reconstructionPoints[index][i].GetPoint();

		feature1.push_back(this->featurePointsList[index][featureID]);
		feature1[i].SetPoint(windage::Vector3(point3D.x, point3D.y, point3D.z));
	}

	std::vector<windage::FeaturePoint>* feature2 = feature;

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(&feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	// pose estimation using Epnp	
	windage::Algorithms::EPnPRANSACestimator poseEstimator;
	poseEstimator.SetReprojectionError(5.0);
	poseEstimator.AttatchCameraParameter(this->cameraParameters[this->caculatedCount]);
	poseEstimator.AttatchReferencePoint(&matchedPoint1);
	poseEstimator.AttatchScenePoint(&matchedPoint2);
	poseEstimator.Calculate();

	int intlierCount = 0;
	for(unsigned int i=0; i<matchedPoint1.size(); i++)
	{
		if(matchedPoint1[i].IsOutlier() == false)
		{
			intlierCount++;
		}
	}

	// reconstruction points
	

	// add & update points

	std::cout << "increament reconstruction : " << intlierCount << " / " << matchedPoint2.size() << std::endl;

	this->caculatedCount++;
	return true;
}

double IncrementalReconstruction::CheckReprojectionError(CvMat **RT, CvMat *pt3D, CvMat **pt2D, int n)
{
	double error = 0.0;
	int size = pt3D->cols;

	CvMat* proj = cvCreateMat(3, 4, CV_64F);
	CvMat* projectionPT = cvCreateMat(3, size, CV_64F);
	CvMat* errorPT = cvCreateMat(3, size, CV_64F);

	for(int i=0; i<n; i++)
	{
		cvMatMul(this->initialCameraParameter->GetIntrinsicMatrix(), RT[i], proj);
		cvMatMul(proj, pt3D, projectionPT);
		cvSub(projectionPT, pt2D[i], errorPT);

		for(int j=0; j<size; j++)
		{
			double ww = 1.0/cvGetReal2D(errorPT, 2, j);
			double x = cvGetReal2D(errorPT, 0, j) * ww;
			double y = cvGetReal2D(errorPT, 1, j) * ww;
			error += sqrt(x*x + y*y);
		}
	}

	cvReleaseMat(&proj);
	cvReleaseMat(&projectionPT);
	cvReleaseMat(&errorPT);

	return error;
}

bool IncrementalReconstruction::BundleAdjustment(int n)
{
	BundleWrapper* bundler = new windage::Reconstruction::BundleWrapper();
	CvMat *pt3D, **pt2D, **RT;
	
	int pointcount = 0;
	for(int i=0; i<n; i++)
		pointcount += (int)this->reconstructionPoints[i].size();

	// set 3d points
	pt3D = cvCreateMat(4, pointcount, CV_64F);
	int count = 0;
	for(int i=0; i<n; i++)
	{
		for(unsigned int j=0; j<this->reconstructionPoints[i].size(); j++)
		{
			windage::Vector4 point3D = this->reconstructionPoints[i][j].GetPoint();
			point3D /= point3D.w;

			cvmSet(pt3D, 0, count, point3D.x);
			cvmSet(pt3D, 1, count, point3D.y);
			cvmSet(pt3D, 2, count, point3D.z);
			cvmSet(pt3D, 3, count, point3D.w);

			count++;
		}
	}

	// set 2d image points
	pt2D = new CvMat*[n];
	for(int i=0; i<n; i++)
	{
		pt2D[i] = cvCreateMat(3, pointcount, CV_64F);
	}

	count = 0;
	for(int i=0; i<n; i++)
	{
		int preCount = 0;
		int j = 0;
		for(; j<count; j++)
		{
			cvmSet(pt2D[i], 0, j, -1.0);
			cvmSet(pt2D[i], 1, j, -1.0);
			cvmSet(pt2D[i], 2, j, 1.0);
			preCount++;
		}
		for(; j<(int)this->reconstructionPoints[i].size()+preCount; j++)
		{
			windage::Vector3 point2D = this->reconstructionPoints[i][j-preCount].GetImagePoint();
			point2D /= point2D.z;
			cvmSet(pt2D[i], 0, j, point2D.x);
			cvmSet(pt2D[i], 1, j, point2D.y);
			cvmSet(pt2D[i], 2, j, point2D.z);

			count++;
		}
		for(; j<pointcount; j++)
		{
			cvmSet(pt2D[i], 0, j, -1.0);
			cvmSet(pt2D[i], 1, j, -1.0);
			cvmSet(pt2D[i], 2, j, 1.0);
		}
	}

	// RT
	RT = new CvMat*[n];
	for(int i=0; i<n; i++)
	{
		RT[i] = cvCreateMat(3, 4, CV_64F);

		CvMat* extrinsic = this->GetCameraParameter(i)->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				cvSetReal2D(RT[i], y, x, cvGetReal2D(extrinsic, y, x));
			}
			cvSetReal2D(RT[i], y, 3, cvGetReal2D(extrinsic, y, 3));
		}
	}

	double error1 = this->CheckReprojectionError(RT, pt3D, pt2D, n);
	bundler->SetParameters(this->initialCameraParameter->GetIntrinsicMatrix(),
							pt3D, pt2D, RT, n, pointcount);
	bundler->Run();
	double error2 = this->CheckReprojectionError(RT, pt3D, pt2D, n);

	// update 3d points
	count = 0;
	for(int i=0; i<n; i++)
	{
		for(unsigned int j=0; j<this->reconstructionPoints[i].size(); j++)
		{
			windage::Vector4 point3D;
			point3D.x = cvmGet(pt3D, 0, count);
			point3D.y = cvmGet(pt3D, 1, count);
			point3D.z = cvmGet(pt3D, 2, count);
			point3D.w = cvmGet(pt3D, 3, count);

			this->reconstructionPoints[i][j].SetPoint(point3D);
			count++;
		}
	}

	// update to calibration 
	for(int i=0; i<n; i++)
	{
		CvMat* extrinsic = this->GetCameraParameter(i)->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				cvSetReal2D(extrinsic, y, x, cvGetReal2D(RT[i], y, x));
			}
			cvSetReal2D(extrinsic, y, 3, cvGetReal2D(RT[i], y, 3));
		}
	}
	
	// release storage
	cvReleaseMat(&pt3D);
	for(int i=0; i<n; i++)
	{
		cvReleaseMat(&pt2D[i]);
		cvReleaseMat(&RT[i]);
	}
	delete [] pt2D;
	delete [] RT;
	delete bundler;

	return true;
}

void IncrementalReconstruction::AttatchFeaturePoint(std::vector<windage::FeaturePoint>* featurePoints)
{
	this->attatchedCount++;
	this->cameraParameters.resize(this->attatchedCount);
	this->featurePointsList.resize(this->attatchedCount);
	this->reconstructionPoints.resize(this->attatchedCount);

	double fx = this->initialCameraParameter->GetParameters()[0];
	double fy = this->initialCameraParameter->GetParameters()[1];
	double cx = this->initialCameraParameter->GetParameters()[2];
	double cy = this->initialCameraParameter->GetParameters()[3];

	this->cameraParameters[this->attatchedCount-1] = new windage::Calibration();
	this->cameraParameters[this->attatchedCount-1]->Initialize(fx, fy, cx, cy);
	for(unsigned int i=0; i<featurePoints->size(); i++)
	{
		this->featurePointsList[this->attatchedCount-1].push_back((*featurePoints)[i]);
	}
}

bool IncrementalReconstruction::Calculate()
{
	if(this->attatchedCount < 2)
		return false;
	if(this->initialCameraParameter == NULL)
		return false;

	this->StereoReconstruction();
	this->BundleAdjustment(2);

	for(int i=2; i<this->attatchedCount; i++)
	{
		this->IncrementReconstruction(&this->featurePointsList[i]);
		this->BundleAdjustment(i+1);
	}

	return true;
}
