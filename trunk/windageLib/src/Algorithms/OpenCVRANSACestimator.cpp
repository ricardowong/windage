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

#include "Algorithms/OpenCVRANSACestimator.h"
using namespace windage;
using namespace windage::Algorithms;

bool OpenCVRANSACestimator::Calculate()
{
	if(this->cameraParameter == NULL)
		return false;
	const int SAMPLE_SIZE = 7;
	int n = (int)this->referencePoints->size();
	if(n < SAMPLE_SIZE)
		return false;
	if(n != (int)this->scenePoints->size())
		return false;

	double fx = this->cameraParameter->GetParameters()[0];
	double fy = this->cameraParameter->GetParameters()[1];
	double cx = this->cameraParameter->GetParameters()[2];
	double cy = this->cameraParameter->GetParameters()[3];

	int idx[SAMPLE_SIZE];
	CvRNG rng = cvRNG(cvGetTickCount());
	int pre_inliers = 0;

	CvMat* samplingRef = cvCreateMat(SAMPLE_SIZE, 3, CV_64FC1);
	CvMat* samplingSce = cvCreateMat(SAMPLE_SIZE, 2, CV_64FC1);

	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* rotationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
	CvMat* extrinsicMatrix = cvCreateMat(4, 4, CV_64FC1);
	cvSetIdentity(extrinsicMatrix);

	windage::Calibration tempCalibration;
	tempCalibration.Initialize(fx, fy, cx, cy);

	std::vector<bool> inliers_checker;		inliers_checker.resize(n);
	std::vector<bool> pre_inlier_checker;	pre_inlier_checker.resize(n);

	int iter = 0;
	int max_iters = this->maxIteration;
	while(iter < max_iters)
	{
		// sampling
		for(int i=0; i<SAMPLE_SIZE; i++)
		{
			int tempIndex = 0;
			bool found = true;
			while(found)
			{
				tempIndex = cvRandInt(&rng) % n;
				found = false;
				for(int j=0; j<i; j++)
				{
					if(idx[j] == tempIndex)
						found = true;
				}
			}
			idx[i] = tempIndex;
		}

		for(int i=0; i<SAMPLE_SIZE; i++)
		{
			windage::Vector3 ref = (*this->referencePoints)[idx[i]].GetPoint();
			windage::Vector3 sce = (*this->scenePoints)[idx[i]].GetPoint();

			CV_MAT_ELEM((*samplingRef), double, i, 0) = ref.x;
			CV_MAT_ELEM((*samplingRef), double, i, 1) = ref.y;
			CV_MAT_ELEM((*samplingRef), double, i, 2) = ref.z;

			CV_MAT_ELEM((*samplingSce), double, i, 0) = sce.x;
			CV_MAT_ELEM((*samplingSce), double, i, 1) = sce.y;
		}

		cvFindExtrinsicCameraParams2(samplingRef, samplingSce,
			tempCalibration.GetIntrinsicMatrix(), tempCalibration.GetDistortionCoefficients(),
			rotationVector, translationVector);

		// convert vector to matrix
		for(int i=0; i < 3; i++) 
		{
			CV_MAT_ELEM((*extrinsicMatrix), double, i, 3) = CV_MAT_ELEM((*translationVector), double, i, 0);
		} 

		cvRodrigues2(rotationVector, rotationMatrix);
		for(int i=0; i<3; i++)
		{
			for(int j=0; j<3; j++)
			{
				CV_MAT_ELEM((*extrinsicMatrix), double, i, j) = CV_MAT_ELEM((*rotationMatrix), double, i, j); 
			}
		}
		tempCalibration.SetExtrinsicMatrix(extrinsicMatrix);

		//count inlier
		int num_inliers = 0;
		for(int i=0; i<n; i++)
		{
			windage::Vector3 ref = (*this->referencePoints)[i].GetPoint();
			windage::Vector3 sce = (*this->scenePoints)[i].GetPoint();

			CvPoint projectionPt = tempCalibration.ConvertWorld2Image(ref.x, ref.y, ref.z);
			windage::Vector2 scePt = windage::Vector2(sce.x, sce.y);
			windage::Vector2 projPt = windage::Vector2(projectionPt.x, projectionPt.y);

			inliers_checker[i] = false;
			if(scePt.getDistance(projPt) < this->reprojectionError)
			{
				num_inliers++;
				inliers_checker[i] = true;
			}
		}

		if(num_inliers > pre_inliers)
		{
			pre_inliers = num_inliers;
			for(int i=0; i<n; i++)
			{
				pre_inlier_checker[i] = inliers_checker[i];
			}

			if(this->confidence > 0)
				max_iters = this->RANSACUpdateNumIters(this->confidence, (double)(n - num_inliers)/(double)n, SAMPLE_SIZE, max_iters);
		}
		
		iter++;
	}

	if(pre_inliers < SAMPLE_SIZE)
		return false;

	// update pose using inlers
	cvReleaseMat(&samplingRef);
	cvReleaseMat(&samplingSce);
	samplingRef = cvCreateMat(pre_inliers, 3, CV_64FC1);
	samplingSce = cvCreateMat(pre_inliers, 2, CV_64FC1);

	int count = 0;
	for(int i=0; i<n; i++)
	{
		if(pre_inlier_checker[i])
		{
			windage::Vector3 ref = (*this->referencePoints)[i].GetPoint();
			windage::Vector3 sce = (*this->scenePoints)[i].GetPoint();

			CV_MAT_ELEM((*samplingRef), double, count, 0) = ref.x;
			CV_MAT_ELEM((*samplingRef), double, count, 1) = ref.y;
			CV_MAT_ELEM((*samplingRef), double, count, 2) = ref.z;

			CV_MAT_ELEM((*samplingSce), double, count, 0) = sce.x;
			CV_MAT_ELEM((*samplingSce), double, count, 1) = sce.y;
			count++;
		}
	}

	cvFindExtrinsicCameraParams2(samplingRef, samplingSce,
			tempCalibration.GetIntrinsicMatrix(), tempCalibration.GetDistortionCoefficients(),
			rotationVector, translationVector);

	// convert vector to matrix
	for(int i=0; i < 3; i++) 
	{
		CV_MAT_ELEM((*extrinsicMatrix), double, i, 3) = CV_MAT_ELEM((*translationVector), double, i, 0);
	} 

	cvRodrigues2(rotationVector, rotationMatrix);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			CV_MAT_ELEM((*extrinsicMatrix), double, i, j) = CV_MAT_ELEM((*rotationMatrix), double, i, j);
		}
	}
	this->cameraParameter->SetExtrinsicMatrix(extrinsicMatrix);

	// count inlier
	int num_inliers = 0;
	for(int i=0; i<n; i++)
	{
		windage::Vector3 ref = (*this->referencePoints)[i].GetPoint();
		windage::Vector3 sce = (*this->scenePoints)[i].GetPoint();

		CvPoint projectionPt = this->cameraParameter->ConvertWorld2Image(ref.x, ref.y, ref.z);
		windage::Vector2 scePt = windage::Vector2(sce.x, sce.y);
		windage::Vector2 projPt = windage::Vector2(projectionPt.x, projectionPt.y);

		if(scePt.getDistance(projPt) < this->reprojectionError)
		{
			num_inliers++;
			(*this->referencePoints)[i].SetOutlier(false);
			(*this->scenePoints)[i].SetOutlier(false);
		}
		else
		{
			(*this->referencePoints)[i].SetOutlier(true);
			(*this->scenePoints)[i].SetOutlier(true);
		}
	}

	cvReleaseMat(&samplingRef);
	cvReleaseMat(&samplingSce);

	cvReleaseMat(&translationVector);
	cvReleaseMat(&rotationVector);
	cvReleaseMat(&rotationMatrix);
	cvReleaseMat(&extrinsicMatrix);
	return true;
}