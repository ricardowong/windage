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
			
			CvScalar color;
			for(int j=0; j<3; j++)
				color.val[j] = (matchedPoint1[i].GetColor().val[j] + matchedPoint2[i].GetColor().val[j])/2.0;
			reconsturctionPoint.SetColor(color);

			// reconstructed point is tracked
			this->featurePointsList[index1][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[index2][matchedPoint2[i].GetRepositoryID()].SetTracked(true);

			reconsturctionPoint.SetObjectID(index1);
			matchedPoint1[i].SetObjectID(index1);
			matchedPoint2[i].SetObjectID(index2);
			reconsturctionPoint.AddFeaturePoint(matchedPoint1[i]);
			reconsturctionPoint.AddFeaturePoint(matchedPoint2[i]);
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
	if(this->caculatedCount < 2)
		return false;

	// find max matching scene
	// temporary index;
	int index = this->caculatedCount - 1;

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

	std::vector<windage::FeaturePoint> matchedPoint1;
	std::vector<windage::FeaturePoint> matchedPoint2;

	this->Matching(&feature1, feature2, &matchedPoint1, &matchedPoint2);

	if(matchedPoint1.size() < 10)
		return false;

	// pose estimation
	this->estimator->AttatchCameraParameter(this->cameraParameters[this->caculatedCount]);
	this->estimator->AttatchReferencePoint(&matchedPoint1);
	this->estimator->AttatchScenePoint(&matchedPoint2);
	this->estimator->Calculate();

	int intlierCount = 0;
	for(unsigned int i=0; i<matchedPoint1.size(); i++)
	{
		if(matchedPoint1[i].IsOutlier() == false)
		{
			intlierCount++;
			int idx = matchedPoint1[i].GetRepositoryID();
			matchedPoint2[i].SetObjectID(this->caculatedCount);
			this->reconstructionPoints[feature1[idx].GetRepositoryID()].AddFeaturePoint(matchedPoint2[i]);
		}
	}

	std::cout << std::endl;
	std::cout << "increament pose estimation (" << index << "-" << this->caculatedCount << ") : " << intlierCount << " / " << matchedPoint2.size() << std::endl;

	// reconstruction points
	matchedPoint1.clear();
	matchedPoint2.clear();
	std::vector<windage::FeaturePoint>* feature = &this->featurePointsList[index];
	this->Matching(feature, feature2, &matchedPoint1, &matchedPoint2);

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
			value = cvGetReal2D(this->cameraParameters[index]->GetExtrinsicMatrix(), y, x);
			cvSetReal2D(extrinsic1, y, x, value);

			value = cvGetReal2D(this->cameraParameters[this->caculatedCount]->GetExtrinsicMatrix(), y, x);
			cvSetReal2D(extrinsic2, y, x, value);
		}
		value = cvGetReal2D(this->cameraParameters[index]->GetExtrinsicMatrix(), y, 3);
		cvSetReal2D(extrinsic1, y, 3, value);

		value = cvGetReal2D(this->cameraParameters[this->caculatedCount]->GetExtrinsicMatrix(), y, 3);
		cvSetReal2D(extrinsic2, y, 3, value);
	}

	CvMat *leftP, *rightP, *nrP3D;
	leftP  = cvCreateMat(3, 1, CV_64F);
	rightP = cvCreateMat(3, 1, CV_64F);
	nrP3D  = cvCreateMat(4, 1, CV_64F);

	int count = 0;
	for(unsigned int i=0; i<matchedPoint1.size(); i++)
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

		CvPoint lpt = this->cameraParameters[index]->ConvertWorld2Image(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2]);
		CvPoint rpt = this->cameraParameters[this->caculatedCount]->ConvertWorld2Image(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2]);

		double errorL = pow((matchedPoint1[i].GetPoint().x - (double)lpt.x), 2) + pow((matchedPoint1[i].GetPoint().y - (double)lpt.y), 2);
		double errorR = pow((matchedPoint2[i].GetPoint().x - (double)rpt.x), 2) + pow((matchedPoint2[i].GetPoint().y - (double)rpt.y), 2);

		if(errorL + errorR < this->reprojectionError * 2)
		{
			windage::ReconstructionPoint reconsturctionPoint;

			reconsturctionPoint.SetPoint(windage::Vector4(nrP3D->data.db[0], nrP3D->data.db[1], nrP3D->data.db[2], 1.0));
			reconsturctionPoint.SetOutlier(false);
			
			CvScalar color;
			for(int j=0; j<3; j++)
				color.val[j] = (matchedPoint1[i].GetColor().val[j] + matchedPoint2[i].GetColor().val[j])/2.0;

			reconsturctionPoint.SetColor(color);

			// reconstructed point is tracked
			this->featurePointsList[index][matchedPoint1[i].GetRepositoryID()].SetTracked(true);
			this->featurePointsList[this->caculatedCount][matchedPoint2[i].GetRepositoryID()].SetTracked(true);

			reconsturctionPoint.SetObjectID(index);
			matchedPoint1[i].SetObjectID(index);
			matchedPoint2[i].SetObjectID(this->caculatedCount);
			reconsturctionPoint.AddFeaturePoint(matchedPoint1[i]);
			reconsturctionPoint.AddFeaturePoint(matchedPoint2[i]);
			this->reconstructionPoints.push_back(reconsturctionPoint);
			count++;
		}
	}

	std::cout << "add increment reconstruction points : " << count << std::endl;
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
	for(unsigned int i=0; i<this->reconstructionPoints.size(); i++)
	{
		windage::Vector4 point3D;
		point3D.x = cvmGet(pt3D, 0, i);
		point3D.y = cvmGet(pt3D, 1, i);
		point3D.z = cvmGet(pt3D, 2, i);
		point3D.w = cvmGet(pt3D, 3, i);

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
		this->BundleAdjustment(2);
	}
	else
	{
		this->IncrementReconstruction();
		this->BundleAdjustment(this->caculatedCount);
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
	this->BundleAdjustment(2);

	for(int i=3; i<this->attatchedCount; i++)
	{
		this->IncrementReconstruction();
		this->BundleAdjustment(i);
	}

	return true;
}

