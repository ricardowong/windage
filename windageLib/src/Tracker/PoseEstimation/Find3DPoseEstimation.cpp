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

#include "Tracker/PoseEstimation/Find3DPoseEstimation.h"

using namespace windage;
double Find3DPoseEstimation::ComputeReprojError(CvPoint2D32f scenePoint, CvPoint3D32f refPoint, CvMat* intrinsicMatrix, CvMat* extrinsicMatrix)
{
	CvMat* point2D = cvCreateMat(3, 1, CV_64FC1);
	CvMat* point3D = cvCreateMat(4, 1, CV_64FC1);
	cvmSet(point3D, 0, 0, refPoint.x);
	cvmSet(point3D, 1, 0, refPoint.y);
	cvmSet(point3D, 2, 0, refPoint.z);
	cvmSet(point3D, 3, 0, 1.0);

	CvMat* temp3D = cvCreateMat(4, 1, CV_64FC1);
	cvMatMul(extrinsicMatrix, point3D, temp3D);

	CvMat* temp2D = cvCreateMat(3, 1, CV_64FC1);
	cvmSet(temp2D, 0, 0, temp3D->data.db[0] / temp3D->data.db[3]);
	cvmSet(temp2D, 1, 0, temp3D->data.db[1] / temp3D->data.db[3]);
	cvmSet(temp2D, 2, 0, temp3D->data.db[2] / temp3D->data.db[3]);

	cvMatMul(intrinsicMatrix, temp2D, point2D);

	double x = cvmGet(point2D, 0, 0) / cvmGet(point2D, 2, 0);
	double y = cvmGet(point2D, 1, 0) / cvmGet(point2D, 2, 0);

	cvReleaseMat(&point2D);
	cvReleaseMat(&point3D);
	cvReleaseMat(&temp2D);
	cvReleaseMat(&temp3D);

	return sqrt((x - scenePoint.x)*(x - scenePoint.x) + (y - scenePoint.y)*(y - scenePoint.y));
}

void ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector, CvMat* extrinsicMatrix)
{
	CvMat* rotationMatrix = cvCreateMat(3, 3, CV_64FC1);

	cvSetZero(extrinsicMatrix);
	int i, j;
    for(i=0; i < 3; i++) 
    {
		cvSetReal2D(extrinsicMatrix, i, 3, cvGetReal1D(translationVector, i));
    } 

	cvRodrigues2(rotationVector, rotationMatrix);
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			cvSetReal2D(extrinsicMatrix, i, j, cvGetReal2D(rotationMatrix, i, j)); 
		}
	}

	cvSetReal2D(extrinsicMatrix, 3, 3, 1.0);

	cvReleaseMat(&rotationMatrix);
}

bool CalculatePose(CvMat* intrinsic, CvMat* distortion, CvMat* extrinsicMatrix, std::vector<windage::Matched3DPoint>* matchedPoints)
{
	CvMat* rotationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	int pointCount = matchedPoints->size();

	CvMat* pImagePoints = cvCreateMat(pointCount, 2, CV_64FC1);
	CvMat* pObjectPoints = cvCreateMat(pointCount, 3, CV_64FC1);

	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, (*matchedPoints)[i].pointScene.x);
		cvSetReal2D(pImagePoints, i, 1, (*matchedPoints)[i].pointScene.y);
	}
	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pObjectPoints, i, 0, (*matchedPoints)[i].pointReference.x);
		cvSetReal2D(pObjectPoints, i, 1, (*matchedPoints)[i].pointReference.y);
		cvSetReal2D(pObjectPoints, i, 2, (*matchedPoints)[i].pointReference.z);
	}

	cvFindExtrinsicCameraParams2(pObjectPoints, pImagePoints, intrinsic, distortion, rotationVector, translationVector);
	ConvertExtrinsicParameter(rotationVector, translationVector, extrinsicMatrix);

	// release matrix
	if(pImagePoints)		cvReleaseMat(&pImagePoints);
	if(pObjectPoints)		cvReleaseMat(&pObjectPoints);
	if(rotationVector)		cvReleaseMat(&rotationVector);
	if(translationVector)	cvReleaseMat(&translationVector);

	return true;
}

double Find3DPoseEstimation::Calculate()
{
	if(this->useRANSAC == false)
	{
		CalculatePose(this->calibration->GetIntrinsicMatrix(), this->calibration->GetDistortionCoefficients(), this->calibration->GetExtrinsicMatrix(), this->matchedPoints);
		return 1.0;
	}
	else
	{
		CvMat* intrinsic = this->calibration->GetIntrinsicMatrix();
		CvMat* extrinsic = cvCreateMat(4, 4, CV_64FC1);
		int numpt = (int)matchedPoints->size();

		const int sample_size = 10;
		if(numpt < sample_size) return -1.0;

		int pre_inlier = -1, num_inliers = 0;
		int max_random_iters = 10, ci = 0;
		const unsigned rng_seed = 0xffffffff;
		CvRNG rng = cvRNG(rng_seed);

		int iter = 100;
		CvMat *tStatus = cvCreateMat(1, numpt, CV_64F);
		CvMat *bStatus = cvCreateMat(1, numpt, CV_64F);

		while(ci < iter)
		{
			int idx[sample_size];
			int snum = 0;
			bool bIn;
			for(int i = 0; i < sample_size; i++ )
			{
				for(int k = 0; k <max_random_iters; k++ )
				{
					idx[i] = cvRandInt(&rng) % numpt;
					bIn = false;
					for(int j = 0; j < snum; j++)
					{
						if(idx[j] == idx[i])
						{
							bIn = true;
							break;
						}
					}

					if(!bIn)
					{
						snum++;
						break; 
					}
				}
			} // done

			std::vector<windage::Matched3DPoint> samplingPoints;
			for(int i=0; i<sample_size; i++)
			{
				samplingPoints.push_back((*this->matchedPoints)[idx[i]]);
			}
			
			// compute pose
			CalculatePose(this->calibration->GetIntrinsicMatrix(), this->calibration->GetDistortionCoefficients(), extrinsic, &samplingPoints);

			// count inliers
			for(int i=0; i<(int)matchedPoints->size(); i++)
			{
				double err = windage::Find3DPoseEstimation::ComputeReprojError((*matchedPoints)[i].pointScene, (*matchedPoints)[i].pointReference, intrinsic, extrinsic);

				if(err < reprojectionThreshold)
				{
					num_inliers++;
					tStatus->data.db[i] = 1.0;
				}
				else
				{
					tStatus->data.db[i] = 0.0;
				}
			}

			if(num_inliers > pre_inlier)
			{
				pre_inlier = num_inliers;
				cvCopy(tStatus, bStatus);

				if(pre_inlier > (double)numpt*0.80)
				{
					// terminate
					ci = iter;
				}
			}

			ci++;
		}

		if(pre_inlier < sample_size)
		{
			return -1.0;
		}

		// recompute RT with inliers
		std::vector<windage::Matched3DPoint> samplingPoints;
		for(int i=0; i<numpt; i++)
		{
			if(bStatus->data.db[i] > 0.0)
			{
				samplingPoints.push_back((*this->matchedPoints)[i]);
			}
		}
		if(samplingPoints.size() < sample_size)
			return -1.0;

		CalculatePose(this->calibration->GetIntrinsicMatrix(), this->calibration->GetDistortionCoefficients(), extrinsic, &samplingPoints);

		// count inliers
		double err = 0.0;
		for(int i=0; i<(int)matchedPoints->size(); i++)
		{
			double localError = windage::Find3DPoseEstimation::ComputeReprojError((*this->matchedPoints)[i].pointScene, (*this->matchedPoints)[i].pointReference, intrinsic, extrinsic);

			if(localError < reprojectionThreshold)
			{
				num_inliers++;
				err += localError;
				(*matchedPoints)[i].isInlier = true;
			}
			else
			{
				(*matchedPoints)[i].isInlier = false;
			}
		}

		// update extrinsic matrix
		CvMat* external = this->calibration->GetExtrinsicMatrix();
		for(int y=0; y<4; y++)
		{
			for(int x=0; x<4; x++)
			{
				cvmSet(external, y, x, cvmGet(extrinsic, y, x));
			}
		}

		cvReleaseMat(&tStatus);
		cvReleaseMat(&bStatus);

		cvReleaseMat(&extrinsic);
		return err/num_inliers;
	}
}

