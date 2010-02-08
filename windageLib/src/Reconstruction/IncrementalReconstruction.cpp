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

#include "Reconstruction/BundleWrapper.h"

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

bool IncrementalReconstruction::StereoReconstruction()
{
	if(this->attatchedCount < 2)
		return false;

	// release before data
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		reconstructionPoints[i].clear();
	}
	
	// matching
	std::vector<windage::FeaturePoint>* feature1 = &featurePointsList[0];
	std::vector<windage::FeaturePoint>* feature2 = &featurePointsList[1];

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	windage::Reconstruction::StereoReconstruction stereo;
	stereo.AttatchBaseCameraParameter(this->GetCameraParameter(0));
	stereo.AttatchUpdateCameraParameter(this->GetCameraParameter(1));

	stereo.AttatchMatchedPoint1(&matchedPoint1);
	stereo.AttatchMatchedPoint2(&matchedPoint2);

	double error = 0.0;
	stereo.CalculateNormalizedPoint();
	stereo.ComputeEssentialMatrixRANSAC(&error);

	std::vector<windage::ReconstructionPoint>* pose3D = stereo.GetReconstructionPoints();
	for(unsigned int i=0;i<pose3D->size(); i++)
	{
		if((*pose3D)[i].IsOutlier() == false)
		{
			windage::ReconstructionPoint reconsturctionPoint1;
			windage::ReconstructionPoint reconsturctionPoint2;

			reconsturctionPoint1.SetPoint((*pose3D)[i].GetPoint());
			reconsturctionPoint2.SetPoint((*pose3D)[i].GetPoint());
			reconsturctionPoint1.SetOutlier(false);
			reconsturctionPoint2.SetOutlier(false);

			reconsturctionPoint1.SetObjectID(0);
			reconsturctionPoint2.SetObjectID(1);
			reconsturctionPoint1.SetFeatureID(matchedPoint1[i].GetRepositoryID());
			reconsturctionPoint2.SetFeatureID(matchedPoint2[i].GetRepositoryID());
			reconsturctionPoint1.SetImagePoint(matchedPoint1[i].GetPoint());
			reconsturctionPoint2.SetImagePoint(matchedPoint2[i].GetPoint());

			// reconstructed point is tracked
			this->featurePointsList[0][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[1][matchedPoint2[i].GetRepositoryID()].SetTracked(true);

			this->reconstructionPoints[0].push_back(reconsturctionPoint1);
			this->reconstructionPoints[1].push_back(reconsturctionPoint2);
		}
	}

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
		feature1.push_back(this->featurePointsList[index][featureID]);
	}

	std::vector<windage::FeaturePoint>* feature2 = feature;

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(&feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	// pose estimation using Epnp	


	// reconstruction points


	// add & update points

	return true;
}

bool IncrementalReconstruction::BundleAdjustment(int n)
{
	BundleWrapper* bundler = new windage::Reconstruction::BundleWrapper();



	return false;
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



	return true;
}
