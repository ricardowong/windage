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

#include "Algorithms/RANSACestimator.h"
#include "Algorithms/OutlierChecker.h"
#include "Reconstruction/BundleWrapper.h"

// Simple linear triangulation method 
// See "Multiple view geometry" written by R.Hartely
void IncrementalReconstruction::LinearTriangulation(CvMat *leftProjectM, CvMat *rightProjectM, CvMat *leftP, CvMat *rightP,CvMat *reconstructedP)
{
	double x,y,x_,y_;
	
	// Linear algorithm AX = 0	// Refer to p298, Multiple Geomery .. /////////////////////////////
	CvMat *A, *U, *VT, *D, *P1, *P2;
	
	// Make up A matrix ///////////////////////////////////////////////////////////////////////////
	A = cvCreateMat(6, 4, CV_64F);
	D = cvCreateMat(6, 4, CV_64F);
	U = cvCreateMat(6, 6, CV_64F);
	VT = cvCreateMat(4, 4, CV_64F);
	
	x  = cvmGet(leftP, 0, 0);
	y  = cvmGet(leftP, 1, 0);
	x_ = cvmGet(rightP, 0, 0); 
	y_ = cvmGet(rightP, 1, 0);
	
	P1 = leftProjectM;
	P2 = rightProjectM;

	cvmSet(A, 0, 0, cvmGet(P1, 2, 0)*x - cvmGet(P1, 0, 0));
	cvmSet(A, 0, 1, cvmGet(P1, 2, 1)*x - cvmGet(P1, 0, 1));
	cvmSet(A, 0, 2, cvmGet(P1, 2, 2)*x - cvmGet(P1, 0, 2));
	cvmSet(A, 0, 3, cvmGet(P1, 2, 3)*x - cvmGet(P1, 0, 3));
	
	cvmSet(A, 1, 0, cvmGet(P1, 2, 0)*y - cvmGet(P1, 1, 0));
	cvmSet(A, 1, 1, cvmGet(P1, 2, 1)*y - cvmGet(P1, 1, 1));
	cvmSet(A, 1, 2, cvmGet(P1, 2, 2)*y - cvmGet(P1, 1, 2));
	cvmSet(A, 1, 3, cvmGet(P1, 2, 3)*y - cvmGet(P1, 1, 3));

	cvmSet(A, 2, 0, cvmGet(P1, 1, 0)*x - cvmGet(P1, 0, 0)*y);
	cvmSet(A, 2, 1, cvmGet(P1, 1, 1)*x - cvmGet(P1, 0, 1)*y);
	cvmSet(A, 2, 2, cvmGet(P1, 1, 2)*x - cvmGet(P1, 0, 2)*y);
	cvmSet(A, 2, 3, cvmGet(P1, 1, 3)*x - cvmGet(P1, 0, 3)*y);
	
	cvmSet(A, 3, 0, cvmGet(P2, 2, 0)*x_ - cvmGet(P2, 0, 0));
	cvmSet(A, 3, 1, cvmGet(P2, 2, 1)*x_ - cvmGet(P2, 0, 1));
	cvmSet(A, 3, 2, cvmGet(P2, 2, 2)*x_ - cvmGet(P2, 0, 2));
	cvmSet(A, 3, 3, cvmGet(P2, 2, 3)*x_ - cvmGet(P2, 0, 3));
	
	cvmSet(A, 4, 0, cvmGet(P2, 2, 0)*y_ - cvmGet(P2, 1, 0));
	cvmSet(A, 4, 1, cvmGet(P2, 2, 1)*y_ - cvmGet(P2, 1, 1));
	cvmSet(A, 4, 2, cvmGet(P2, 2, 2)*y_ - cvmGet(P2, 1, 2));
	cvmSet(A, 4, 3, cvmGet(P2, 2, 3)*y_ - cvmGet(P2, 1, 3));

	cvmSet(A, 5, 0, cvmGet(P2, 1, 0)*x_ - cvmGet(P2, 0, 0)*y_);
	cvmSet(A, 5, 1, cvmGet(P2, 1, 1)*x_ - cvmGet(P2, 0, 1)*y_);
	cvmSet(A, 5, 2, cvmGet(P2, 1, 2)*x_ - cvmGet(P2, 0, 2)*y_);
	cvmSet(A, 5, 3, cvmGet(P2, 1, 3)*x_ - cvmGet(P2, 0, 3)*y_);
	
	// Solve AX = 0 by using p564, SVD = UWVT /////////////////////////////////////////////////////
	cvSVD(A, D, U, VT, CV_SVD_V_T);
	
	// Return the values //////////////////////////////////////////////////////////////////////////
	for(int i=0; i<4; i++)
		cvmSet(reconstructedP, i, 0, cvmGet(VT, 3, i));
	
	cvReleaseMat(&A);
	cvReleaseMat(&D);
	cvReleaseMat(&U);
	cvReleaseMat(&VT);
}

bool IncrementalReconstruction::Matching(std::vector<windage::FeaturePoint>* feature1, std::vector<windage::FeaturePoint>* feature2, std::vector<windage::FeaturePoint>* matchedPoint1, std::vector<windage::FeaturePoint>* matchedPoint2)
{
	searchtree->Training(feature1);
	for(unsigned int i=0; i<feature2->size(); i++)
	{
		int index = searchtree->Matching((*feature2)[i]);
		if(index >= 0)
		{
			matchedPoint1->push_back((*feature1)[index]);
			matchedPoint2->push_back((*feature2)[i]);
			(*matchedPoint1)[matchedPoint1->size()-1].SetRepositoryID(index);
			(*matchedPoint2)[matchedPoint1->size()-1].SetRepositoryID(i);
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
	reconstructionPoints.clear();

	// matching
	std::vector<windage::FeaturePoint>* feature1 = &featurePointsList[index1];
	std::vector<windage::FeaturePoint>* feature2 = &featurePointsList[index2];

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	windage::Reconstruction::StereoReconstruction stereo;
	stereo.SetReprojectionError(this->reprojectionError);
	stereo.SetConfidence(this->confidence);
	stereo.SetMaxIteration(this->maxIteration);

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
			windage::ReconstructionPoint reconsturctionPoint;

			reconsturctionPoint.SetPoint((*pose3D)[i].GetPoint());
			reconsturctionPoint.SetOutlier(false);
			
			// reconstructed point is tracked
			int index = (int)this->reconstructionPoints.size();
			this->featurePointsList[index1][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[index2][matchedPoint2[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[index1][matchedPoint1[i].GetRepositoryID()].SetRepositoryID(index);
			this->featurePointsList[index2][matchedPoint2[i].GetRepositoryID()].SetRepositoryID(index);

			reconsturctionPoint.SetObjectID(index1);
			matchedPoint1[i].SetObjectID(index1);
			matchedPoint2[i].SetObjectID(index2);
			reconsturctionPoint.AddFeaturePoint(matchedPoint1[i]);
			reconsturctionPoint.AddFeaturePoint(matchedPoint2[i]);
			reconsturctionPoint.SetColor(matchedPoint1[i].GetColor());
			this->reconstructionPoints.push_back(reconsturctionPoint);
			count++;
		}
	}

	std::cout << std::endl;
	std::cout << "stereo reconstruction : " << stereo.GetInlierCount() << " / " << matchedPoint2.size() << std::endl;
	std::cout << "add stereo reconstruction points : " << count << std::endl;
	std::cout << std::endl;

	this->caculatedCount = 2;
	return true;
}

bool IncrementalReconstruction::IncrementReconstruction()
{
	const int MINIMUM_MATCHING_COUNT = 9;
	if(this->caculatedCount < 2)
		return false;

	// find best matching scene
	int index = this->caculatedCount - 1;
/*
	int maxCount = 0;
	for(int k=0; k<this->caculatedCount; k++)
	{
		std::vector<windage::FeaturePoint> reconstructedFeature;
		reconstructedFeature.clear();
		for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
		{
			windage::Vector4 point3D = this->reconstructionPoints[i].GetPoint();
			std::vector<windage::FeaturePoint>* features = this->reconstructionPoints[i].GetFeatureList();
			for(unsigned int j=0; j<features->size(); j++)
			{
				if((*features)[j].GetObjectID() == k)
				{
					reconstructedFeature.push_back((*features)[j]);
					int idx = reconstructedFeature.size() - 1;
					reconstructedFeature[idx].SetRepositoryID(i);
					reconstructedFeature[idx].SetPoint(windage::Vector3(point3D.x, point3D.y, point3D.z));
				}
			}		
		}

		if((int)reconstructedFeature.size() > MINIMUM_MATCHING_COUNT)
		{
			int matchedCount = this->MatchingCount(&reconstructedFeature, &this->featurePointsList[this->caculatedCount]);
			if(maxCount < matchedCount)
			{
				maxCount = matchedCount;
				index = k;
			}
		}
	}
	std::cout << std::endl;
	std::cout << "best matching image index : " << index  << "(" << maxCount << ")" << std::endl;
//*/

	// matching
	std::vector<windage::FeaturePoint> feature1;
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		windage::Vector4 point3D = this->reconstructionPoints[i].GetPoint();
		std::vector<windage::FeaturePoint>* features = this->reconstructionPoints[i].GetFeatureList();
		for(unsigned int j=0; j<features->size(); j++)
		{
			if((*features)[j].GetObjectID() == index)
			{
				feature1.push_back((*features)[j]);
				int idx = feature1.size() - 1;
				feature1[idx].SetRepositoryID(i);
				feature1[idx].SetPoint(windage::Vector3(point3D.x, point3D.y, point3D.z));
			}
		}		
	}
	std::vector<windage::FeaturePoint>* feature2 = &this->featurePointsList[this->caculatedCount];
	if((int)feature1.size() < MINIMUM_MATCHING_COUNT)
	{
		this->caculatedCount++;
		return false;
	}

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;
	this->Matching(&feature1, feature2, &matchedPoint1, &matchedPoint2);

	if((int)matchedPoint1.size() < MINIMUM_MATCHING_COUNT)
	{
		this->caculatedCount++;
		return false;
	}

	// pose estimation
	int repeat = 0;
	int intlierCount = 0;
	while(intlierCount < MINIMUM_MATCHING_COUNT && repeat++ < MINIMUM_MATCHING_COUNT)
	{
		this->estimator->AttatchCameraParameter(this->cameraParameters[this->caculatedCount]);
		this->estimator->AttatchReferencePoint(&matchedPoint1);
		this->estimator->AttatchScenePoint(&matchedPoint2);
		this->estimator->Calculate();

		for(unsigned int i=0; i<matchedPoint1.size(); i++)
		{
			if(matchedPoint1[i].IsOutlier() == false)
			{
				intlierCount++;
//				int idx = matchedPoint1[i].GetRepositoryID();
//				matchedPoint2[i].SetObjectID(this->caculatedCount);
//				this->reconstructionPoints[feature1[idx].GetRepositoryID()].AddFeaturePoint(matchedPoint2[i]);
			}
		}

		std::cout << std::endl;
		std::cout << "increament pose estimation (" << index << "-" << this->caculatedCount << ") : "
				  << intlierCount << " / " << matchedPoint2.size() << " (" << (double)intlierCount/(double)matchedPoint2.size() << ")" << std::endl;
	}

	// reconstruction points
	matchedPoint1.clear();
	matchedPoint2.clear();
	std::vector<windage::FeaturePoint>* feature = &this->featurePointsList[index];
	this->Matching(feature, feature2, &matchedPoint1, &matchedPoint2);

	// outlier rejection
//*
	double ratio = 3.0;
	windage::Algorithms::RANSACestimator hEstimator;
	windage::Algorithms::OutlierChecker checker;
	hEstimator.SetReprojectionError(this->reprojectionError * ratio);

	hEstimator.AttatchReferencePoint(&matchedPoint1);	
	hEstimator.AttatchScenePoint(&matchedPoint2);
	hEstimator.Calculate();

	checker.SetReprojectionError(this->reprojectionError * ratio);
	checker.AttatchEstimator(&hEstimator);
	checker.Calculate();
//*/

	// add 3d points
	CvMat* inverseIntrinsic = cvCreateMat(3, 3, CV_64F);
	cvInvert(this->initialCameraParameter->GetIntrinsicMatrix(), inverseIntrinsic);
	CvMat* extrinsic1 = cvCreateMat(3, 4, CV_64F);
	CvMat* extrinsic2 = cvCreateMat(3, 4, CV_64F);
	double value;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			value = CV_MAT_ELEM((*this->cameraParameters[index]->GetExtrinsicMatrix()), double, y, x);
			CV_MAT_ELEM((*extrinsic1), double, y, x) = value;

			value = CV_MAT_ELEM((*this->cameraParameters[this->caculatedCount]->GetExtrinsicMatrix()), double, y, x);
			CV_MAT_ELEM((*extrinsic2), double, y, x) = value;
		}
		value = CV_MAT_ELEM((*this->cameraParameters[index]->GetExtrinsicMatrix()), double, y, 3);
		CV_MAT_ELEM((*extrinsic1), double, y, 3) = value;

		value = CV_MAT_ELEM((*this->cameraParameters[this->caculatedCount]->GetExtrinsicMatrix()), double, y, 3);
		CV_MAT_ELEM((*extrinsic2), double, y, 3) = value;
	}

	CvMat *leftP, *rightP, *nrP3D;
	leftP  = cvCreateMat(3, 1, CV_64F);
	rightP = cvCreateMat(3, 1, CV_64F);
	nrP3D  = cvCreateMat(4, 1, CV_64F);

	int addCount = 0;
	int count = 0;
	for(unsigned int i=0; i<matchedPoint1.size(); i++)
	{
		if(matchedPoint1[i].IsOutlier() == false)
		{
			cvmSet(leftP, 0, 0, matchedPoint1[i].GetPoint().x);
			cvmSet(leftP, 1, 0, matchedPoint1[i].GetPoint().y);
			cvmSet(leftP, 2, 0, 1.0);

			cvmSet(rightP, 0, 0, matchedPoint2[i].GetPoint().x);
			cvmSet(rightP, 1, 0, matchedPoint2[i].GetPoint().y);
			cvmSet(rightP, 2, 0, 1.0);

			// normalized
			cvMatMul(inverseIntrinsic, leftP, leftP);
			cvMatMul(inverseIntrinsic, rightP, rightP);

			LinearTriangulation(extrinsic1, extrinsic2, leftP, rightP, nrP3D);

			nrP3D->data.db[0] /= nrP3D->data.db[3];
			nrP3D->data.db[1] /= nrP3D->data.db[3];
			nrP3D->data.db[2] /= nrP3D->data.db[3];
			nrP3D->data.db[3] /= nrP3D->data.db[3];

			windage::Vector2 lpt = this->cameraParameters[index]->ConvertWorld2Imaged(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2]);
			windage::Vector2 rpt = this->cameraParameters[this->caculatedCount]->ConvertWorld2Imaged(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2]);

			double errorL = pow((matchedPoint1[i].GetPoint().x - lpt.x), 2) + pow((matchedPoint1[i].GetPoint().y - lpt.y), 2);
			double errorR = pow((matchedPoint2[i].GetPoint().x - rpt.x), 2) + pow((matchedPoint2[i].GetPoint().y - rpt.y), 2);
			errorL = sqrt(errorL);
			errorR = sqrt(errorR);
//			std::cout << this->reprojectionError << " : " <<  errorL << ", " << errorR << std::endl;
			if(errorL + errorR < this->reprojectionError * 2.0)
			{
				windage::ReconstructionPoint reconsturctionPoint;

				reconsturctionPoint.SetPoint(windage::Vector4(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2], 1.0));
				reconsturctionPoint.SetOutlier(false);
				
				CvScalar color;
				for(int j=0; j<3; j++)
					color.val[j] = (matchedPoint1[i].GetColor().val[j] + matchedPoint2[i].GetColor().val[j])/2.0;

				reconsturctionPoint.SetColor(color);

				// reconstructed point is tracked
				if(this->featurePointsList[index][matchedPoint1[i].GetRepositoryID()].IsTracked())
				{
					int idx = this->featurePointsList[index][matchedPoint1[i].GetRepositoryID()].GetRepositoryID();
					windage::Vector4 worldPT = this->reconstructionPoints[idx].GetPoint();

					// check false matching
					windage::Vector2 lpt = this->cameraParameters[index]->ConvertWorld2Imaged(worldPT.x, worldPT.y, worldPT.z);
					windage::Vector2 rpt = this->cameraParameters[this->caculatedCount]->ConvertWorld2Imaged(worldPT.x, worldPT.y, worldPT.z);

					double errorL = pow((matchedPoint1[i].GetPoint().x - lpt.x), 2) + pow((matchedPoint1[i].GetPoint().y - lpt.y), 2);
					double errorR = pow((matchedPoint2[i].GetPoint().x - rpt.x), 2) + pow((matchedPoint2[i].GetPoint().y - rpt.y), 2);

					errorL = sqrt(errorL);
					errorR = sqrt(errorR);
					if(errorL + errorR < this->reprojectionError * 2.0)
					{
						this->featurePointsList[this->caculatedCount][matchedPoint2[i].GetRepositoryID()].SetTracked(true);
						this->featurePointsList[this->caculatedCount][matchedPoint2[i].GetRepositoryID()].SetRepositoryID(idx);

						matchedPoint2[i].SetObjectID(this->caculatedCount);

						windage::Vector4 pt1 = reconsturctionPoint.GetPoint();
						windage::Vector4 pt2 = this->reconstructionPoints[idx].GetPoint();
						double error = pt1.getDistance(pt2);

						this->reconstructionPoints[idx].AddFeaturePoint(matchedPoint2[i]);
						addCount++;
					}
				}
				else
				{
					int idx = this->reconstructionPoints.size();
					this->featurePointsList[index][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
					this->featurePointsList[this->caculatedCount][matchedPoint2[i].GetRepositoryID()].SetTracked(true);

					this->featurePointsList[index][matchedPoint1[i].GetRepositoryID()].SetRepositoryID(idx);
					this->featurePointsList[this->caculatedCount][matchedPoint2[i].GetRepositoryID()].SetRepositoryID(idx);

					reconsturctionPoint.SetObjectID(index);
					matchedPoint1[i].SetObjectID(index);
					matchedPoint2[i].SetObjectID(this->caculatedCount);
					reconsturctionPoint.AddFeaturePoint(matchedPoint1[i]);
					reconsturctionPoint.AddFeaturePoint(matchedPoint2[i]);

					this->reconstructionPoints.push_back(reconsturctionPoint);
					count++;
				}
			}
		}
	}

	std::cout << "add increment reconstruction points : " << addCount << " + " << count << " / " << matchedPoint1.size()
															<< " (" << (addCount + count) / (double)matchedPoint1.size() << ")" <<  std::endl;
	std::cout << "reconstructed all points : " << this->reconstructionPoints.size() << std::endl;
	std::cout << std::endl;

	cvReleaseMat(&inverseIntrinsic);
	cvReleaseMat(&leftP);
	cvReleaseMat(&rightP);
	cvReleaseMat(&nrP3D);

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
			double ww = 1.0/CV_MAT_ELEM((*errorPT), double, 2, j);
			double x = CV_MAT_ELEM((*errorPT), double, 0, j) * ww;
			double y = CV_MAT_ELEM((*errorPT), double, 1, j) * ww;
			error += sqrt(x*x + y*y);
		}
	}

	cvReleaseMat(&proj);
	cvReleaseMat(&projectionPT);
	cvReleaseMat(&errorPT);

	return error;
}

bool IncrementalReconstruction::BundleAdjustment()
{
	int n = this->caculatedCount;
	BundleWrapper* bundler = new windage::Reconstruction::BundleWrapper();
	CvMat *pt3D, **pt2D, **RT;
	
	int pointcount = (int)this->reconstructionPoints.size();

	// set 3d points
	pt3D = cvCreateMat(4, pointcount, CV_64F);
	for(unsigned int j=0; j<this->reconstructionPoints.size(); j++)
	{
		windage::Vector4 point3D = this->reconstructionPoints[j].GetPoint();
		point3D /= point3D.w;

		cvmSet(pt3D, 0, j, point3D.x);
		cvmSet(pt3D, 1, j, point3D.y);
		cvmSet(pt3D, 2, j, point3D.z);
		cvmSet(pt3D, 3, j, point3D.w);
	}

	// set 2d image points
	pt2D = new CvMat*[n];
	for(int i=0; i<n; i++)
	{
		pt2D[i] = cvCreateMat(3, pointcount, CV_64F);

		for(int j=0; j<pointcount; j++)
		{
			cvmSet(pt2D[i], 0, j, -1.0);
			cvmSet(pt2D[i], 1, j, -1.0);
			cvmSet(pt2D[i], 2, j, 1.0);
		}
	}

	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* features = this->reconstructionPoints[i].GetFeatureList();

		for(unsigned int j=0; j<features->size(); j++)
		{
			int idx = (*features)[j].GetObjectID();
			windage::Vector3 imagePoint = (*features)[j].GetPoint();
			
			cvmSet(pt2D[idx], 0, i, imagePoint.x);
			cvmSet(pt2D[idx], 1, i, imagePoint.y);
			cvmSet(pt2D[idx], 2, i, imagePoint.z);
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
				CV_MAT_ELEM((*RT[i]), double, y, x) = CV_MAT_ELEM((*extrinsic), double, y, x);
			}
			CV_MAT_ELEM((*RT[i]), double, y, 3) = CV_MAT_ELEM((*extrinsic), double, y, 3);
		}
	}

	double error1 = this->CheckReprojectionError(RT, pt3D, pt2D, n);
	bundler->SetParameters(this->initialCameraParameter->GetIntrinsicMatrix(),
							pt3D, pt2D, RT, n, pointcount);
	bundler->Run();
	double error2 = this->CheckReprojectionError(RT, pt3D, pt2D, n);

	// update 3d points
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		windage::Vector4 point3D;
		point3D.x = cvmGet(pt3D, 0, i);
		point3D.y = cvmGet(pt3D, 1, i);
		point3D.z = cvmGet(pt3D, 2, i);
		point3D.w = cvmGet(pt3D, 3, i);
		point3D /= point3D.w;

		this->reconstructionPoints[i].SetPoint(point3D);
	}

	// update to calibration 
	for(int i=0; i<n; i++)
	{
		CvMat* extrinsic = this->GetCameraParameter(i)->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				CV_MAT_ELEM((*extrinsic), double, y, x) = CV_MAT_ELEM((*RT[i]), double, y, x);
			}
			CV_MAT_ELEM((*extrinsic), double, y, 3) = CV_MAT_ELEM((*RT[i]), double, y, 3);
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

bool IncrementalReconstruction::BundleAdjustment(int startIndex, int n)
{
	BundleWrapper* bundler = new windage::Reconstruction::BundleWrapper();
	CvMat *pt3D, **pt2D, **RT;
	
	int pointcount = 0;
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = this->reconstructionPoints[i].GetFeatureList();
		bool found = false;
		for(unsigned int j=0; j<featureList->size() && found==false; j++)
		{
			int objectID = (*featureList)[j].GetObjectID();
			if(startIndex <= objectID && objectID < startIndex + n)
			{
				pointcount++;
				found = true;
			}
		}
	}

	// set 3d points
	pt3D = cvCreateMat(4, pointcount, CV_64F);

	int index = 0;
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = this->reconstructionPoints[i].GetFeatureList();
		bool found = false;
		for(unsigned int j=0; j<featureList->size(); j++)
		{
			int objectID = (*featureList)[j].GetObjectID();
			if(startIndex <= objectID && objectID < startIndex + n)
			{
				windage::Vector4 point3D = this->reconstructionPoints[i].GetPoint();
				point3D /= point3D.w;

				cvmSet(pt3D, 0, index, point3D.x);
				cvmSet(pt3D, 1, index, point3D.y);
				cvmSet(pt3D, 2, index, point3D.z);
				cvmSet(pt3D, 3, index, point3D.w);
				found = true;
			}
		}

		if(found)
			index++;
	}

	// set 2d image points
	pt2D = new CvMat*[n];
	for(int i=0; i<n; i++)
	{
		pt2D[i] = cvCreateMat(3, pointcount, CV_64F);

		for(int j=0; j<pointcount; j++)
		{
			cvmSet(pt2D[i], 0, j, -1.0);
			cvmSet(pt2D[i], 1, j, -1.0);
			cvmSet(pt2D[i], 2, j, 1.0);
		}
	}

	index = 0;
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = this->reconstructionPoints[i].GetFeatureList();
		bool found = false;
		for(unsigned int j=0; j<featureList->size(); j++)
		{
			int objectID = (*featureList)[j].GetObjectID();
			if(startIndex <= objectID && objectID < startIndex + n)
			{
				int idx = objectID - startIndex;
				windage::Vector3 imagePoint = (*featureList)[j].GetPoint();
				
				cvmSet(pt2D[idx], 0, index, imagePoint.x);
				cvmSet(pt2D[idx], 1, index, imagePoint.y);
				cvmSet(pt2D[idx], 2, index, imagePoint.z);
				found = true;
			}
		}

		if(found)
			index++;
	}

	// RT
	RT = new CvMat*[n];
	for(int i=0; i<n; i++)
	{
		RT[i] = cvCreateMat(3, 4, CV_64F);

		CvMat* extrinsic = this->GetCameraParameter(startIndex + i)->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				CV_MAT_ELEM((*RT[i]), double, y, x) = CV_MAT_ELEM((*extrinsic), double, y, x);
			}
			CV_MAT_ELEM((*RT[i]), double, y, 3) = CV_MAT_ELEM((*extrinsic), double, y, 3);
		}
	}

	double error1 = this->CheckReprojectionError(RT, pt3D, pt2D, n);
	bundler->SetParameters(this->initialCameraParameter->GetIntrinsicMatrix(),
							pt3D, pt2D, RT, n, pointcount);
	bundler->Run();
	double error2 = this->CheckReprojectionError(RT, pt3D, pt2D, n);

	// update 3d points
	index = 0;
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = this->reconstructionPoints[i].GetFeatureList();
		bool found = false;
		for(unsigned int j=0; j<featureList->size(); j++)
		{
			int objectID = (*featureList)[j].GetObjectID();
			if(startIndex <= objectID && objectID < startIndex + n)
			{
				windage::Vector4 point3D;
				point3D.x = cvmGet(pt3D, 0, index);
				point3D.y = cvmGet(pt3D, 1, index);
				point3D.z = cvmGet(pt3D, 2, index);
				point3D.w = cvmGet(pt3D, 3, index);
				point3D /= point3D.w;

				this->reconstructionPoints[i].SetPoint(point3D);
				found = true;
			}
		}

		if(found)
			index++;
	}

	// update to calibration 
	for(int i=0; i<n; i++)
	{
		CvMat* extrinsic = this->GetCameraParameter(startIndex + i)->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				CV_MAT_ELEM((*extrinsic), double, y, x) = CV_MAT_ELEM((*RT[i]), double, y, x);
			}
			CV_MAT_ELEM((*extrinsic), double, y, 3) = CV_MAT_ELEM((*RT[i]), double, y, 3);
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

bool IncrementalReconstruction::CalculateStep(int step)
{
	int n = step;
	if(n <= 1)
		return false;
	if(n > this->attatchedCount)
		return false;
	if(this->attatchedCount < 2)
		return false;
	if(this->initialCameraParameter == NULL)
		return false;

	if(n == 2)
	{
		this->StereoReconstruction();
//		this->BundleAdjustment(2);
	}
	else
	{
		this->caculatedCount = step-1;
		this->IncrementReconstruction();
//		this->BundleAdjustment(this->caculatedCount);
	}

	return true;
}

bool IncrementalReconstruction::CalculateAll()
{
	if(this->attatchedCount < 2)
		return false;
	if(this->initialCameraParameter == NULL)
		return false;

	this->StereoReconstruction();
	this->BundleAdjustment();

	for(int i=3; i<=this->attatchedCount; i++)
	{
		this->IncrementReconstruction();

		int STEP = MIN(i, BUNDLE_STEP);
		this->BundleAdjustment(i-STEP, STEP);

		if(i % 10 == 0)
			this->BundleAdjustment();
	}
	this->BundleAdjustment();

	return true;
}

bool IncrementalReconstruction::UpdateColor()
{
	int count = (unsigned)this->reconstructionPoints.size();
	for(int i=0; i<count; i++)
	{
		std::vector<windage::FeaturePoint>* featureList = this->reconstructionPoints[i].GetFeatureList();
		int featureCount = featureList->size();

		CvScalar color = cvScalarAll(0);
		for(int j=0; j<featureCount; j++)
		{
			for(int k=0; k<3; k++)
				color.val[k] += (*featureList)[j].GetColor().val[k];
		}
		for(int k=0; k<3; k++)
			color.val[k] /= (double)featureCount;

		this->reconstructionPoints[i].SetColor(color);
	}

	return true;
}

bool IncrementalReconstruction::ResizeScale(double scale)
{
	windage::Vector4 center;
	int count = (unsigned)this->reconstructionPoints.size();
	for(int i=0; i<count; i++)
	{
		center += this->reconstructionPoints[i].GetPoint();
	}
	center /= (double)count;

	double distance = 0.0;
	for(int i=0; i<count; i++)
	{
		distance += center.getDistance(this->reconstructionPoints[i].GetPoint());
	}
	distance /= (double)count;
	scale = scale / distance;

	// points translation
	for(int i=0; i<count; i++)
	{
		windage::Vector4 tempPoint = this->reconstructionPoints[i].GetPoint();
		tempPoint = (tempPoint - center) * scale;
		tempPoint.w = 1.0;
		this->reconstructionPoints[i].SetPoint(tempPoint);
	}

	// camera translation
	for(unsigned int i=0; i<this->cameraParameters.size(); i++)
	{
		CvScalar position = this->cameraParameters[i]->GetCameraPosition();

		for(int k=0; k<3; k++)
		{
			position.val[k] -= center.v[k];
			position.val[k] *= scale;
		}

		this->cameraParameters[i]->SetCameraPosition(position);
	}

	return true;
}