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

#include "Tracker/PoseEstimation/FindEpnpPoseEstimation.h"
#include "Tracker/PoseEstimation/epnp.h"

using namespace windage;
double FindEpnpPoseEstimation::Calculate()
{
	int k = this->matchedPoints->size();
	double _ret = -1.0;
	double cameraPose[16];
	if(k>5)
	{
		_ret = RunEpnpRANSAC(this->matchedPoints, cameraPose);
	}
	return _ret;
}

double FindEpnpPoseEstimation::RunEpnpRANSAC(std::vector<Matched3DPoint>* matchedPoints, double* cameraPose)
{
	double _rerror = -1.0;
	int numpt = (int)matchedPoints->size();
	if(numpt < 5) return -1;

	double fx = this->calibration->GetParameters()[0];
	double fy = this->calibration->GetParameters()[1];
	double cx = this->calibration->GetParameters()[2];
	double cy = this->calibration->GetParameters()[3];

	CvMat *intric = cvCreateMat(3, 3, CV_64F);
	cvmSetZero(intric);
	cvmSet(intric, 0, 0, fx);
	cvmSet(intric, 1, 1, fy);
	cvmSet(intric, 0, 2, cx);
	cvmSet(intric, 1, 2, cy);
	cvmSet(intric, 2, 2, 1.0);

	// Pose estimation using PnP alogrithm from EPFL
	epnp *_epnp = new epnp;
	_epnp->set_internal_parameters(cx, cy, fx, fy);

	int sample_size = 5, pre_inlier = -1, num_inliers = 0;
	int max_random_iters = 10, ci = 0;
	const unsigned rng_seed = 0xffffffff;
	CvRNG rng = cvRNG(rng_seed);
	double pre_error = 10000, small_err = 10000.0, _mR[3][3], _mt[3];

	int iter = 300;
	CvMat *tStatus = cvCreateMat(1, numpt, CV_64F);
	CvMat *bStatus = cvCreateMat(1, numpt, CV_64F);

	while(ci < iter)
	{
		int *idx = new int[sample_size];
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

		_epnp->set_maximum_number_of_correspondences(sample_size);
		_epnp->reset_correspondences();

		for(int i=0; i<sample_size; i++)
		{
			double _X, _Y, _Z, _u, _v;
			_X = (*matchedPoints)[idx[i]].pointReference.x;
			_Y = (*matchedPoints)[idx[i]].pointReference.y;
			_Z = (*matchedPoints)[idx[i]].pointReference.z;
			_u = (*matchedPoints)[idx[i]].pointScene.x;
			_v = (*matchedPoints)[idx[i]].pointScene.y;

			_epnp->add_correspondence(_X, _Y, _Z, _u, _v);
		}
		delete [] idx;

		// compute pose
		double _R[3][3], _t[3];
		_rerror  = _epnp->compute_pose(_R, _t);

		// count inliers
		num_inliers = 0;
		CvMat *Proj = cvCreateMat(3, 4, CV_64F);
		for(int p=0; p<3; p++)
		{
			for(int q=0; q<3; q++)
			{
				cvmSet(Proj, p, q, _R[p][q]);
			}
			cvmSet(Proj, p, 3, _t[p]);
		}
		cvMatMul(intric, Proj, Proj);

		double x, y, err;
		CvMat *p2d = cvCreateMat(3, 1, CV_64F);
		CvMat *p3d = cvCreateMat(3, 1, CV_64F);
		for(int i=0; i<(int)matchedPoints->size(); i++)
		{
			p3d->data.db[0] = (*matchedPoints)[i].pointReference.x;
			p3d->data.db[1] = (*matchedPoints)[i].pointReference.y;
			p3d->data.db[2] = (*matchedPoints)[i].pointReference.z;

			cvMatMul(Proj, p3d, p2d);
			x = p2d->data.db[0]/p2d->data.db[2];
			y = p2d->data.db[1]/p2d->data.db[2];
			err = ((*matchedPoints)[i].pointScene.x - x)*((*matchedPoints)[i].pointScene.x - x) + ((*matchedPoints)[i].pointScene.y - y)*((*matchedPoints)[i].pointScene.y - y);
				
			if(cvSqrt(err) < 2.0)
			{
				num_inliers++;
				tStatus->data.db[i] = 1.0;
			}
			else
			{
				tStatus->data.db[i] = 0.0;
			}
		}
		cvReleaseMat(&Proj);
		cvReleaseMat(&p2d);
		cvReleaseMat(&p3d);


		if(num_inliers > pre_inlier)
		{
			pre_inlier = num_inliers;
			cvCopy(tStatus, bStatus);

			if(pre_inlier > (double)numpt*0.9)
			{
				break;
			}
		}

		ci++;
	}
	delete _epnp;


	if(pre_inlier < 5)
	{
		printf("Few inliers in epnp. \n");
		return -1;
	}


	// recompute RT with inliers
	epnp *_nepnp = new epnp;
	_nepnp->set_internal_parameters(cx, cy, fx, fy);
	_nepnp->set_maximum_number_of_correspondences(pre_inlier);
	_nepnp->reset_correspondences();

	for(int i=0; i<numpt; i++)
	{
		if(bStatus->data.db[i] > 0)
		{
			double _X, _Y, _Z, _u, _v;
			_X = (*matchedPoints)[i].pointReference.x;
			_Y = (*matchedPoints)[i].pointReference.y;
			_Z = (*matchedPoints)[i].pointReference.z;
			_u = (*matchedPoints)[i].pointScene.x;
			_v = (*matchedPoints)[i].pointScene.y;

			_nepnp->add_correspondence(_X, _Y, _Z, _u, _v);
		}
	}
	_rerror  = _nepnp->compute_pose(_mR, _mt);

	// count inliers
	num_inliers = 0;
	CvMat *Proj = cvCreateMat(3, 4, CV_64F);
	for(int p=0; p<3; p++)
	{
		for(int q=0; q<3; q++)
		{
			cvmSet(Proj, p, q, _mR[p][q]);
		}
		cvmSet(Proj, p, 3, _mt[p]);
	}
	cvMatMul(intric, Proj, Proj);

	double x, y, err;
	CvMat *p2d = cvCreateMat(3, 1, CV_64F);
	CvMat *p3d = cvCreateMat(3, 1, CV_64F);
	for(int i=0; i<(int)matchedPoints->size(); i++)
	{
		p3d->data.db[0] = (*matchedPoints)[i].pointReference.x;
		p3d->data.db[1] = (*matchedPoints)[i].pointReference.y;
		p3d->data.db[2] = (*matchedPoints)[i].pointReference.z;

		cvMatMul(Proj, p3d, p2d);
		x = p2d->data.db[0]/p2d->data.db[2];
		y = p2d->data.db[1]/p2d->data.db[2];
		err = ((*matchedPoints)[i].pointScene.x - x)*((*matchedPoints)[i].pointScene.x - x) + ((*matchedPoints)[i].pointScene.y - y)*((*matchedPoints)[i].pointScene.y - y);

		if(cvSqrt(err) < 2.0)
		{
			num_inliers++;
			(*matchedPoints)[i].isInlier = true;
		}
		else
		{
			(*matchedPoints)[i].isInlier = false;
		}
	}
	cvReleaseMat(&Proj);
	cvReleaseMat(&p2d);
	cvReleaseMat(&p3d);

	cvReleaseMat(&tStatus);
	cvReleaseMat(&bStatus);
	

	if(num_inliers < 5) _rerror = -1.0;

	for(int p=0; p<3; p++)
	{
		for(int q=0; q<3; q++)
		{
			cameraPose[p*4+q] = _mR[p][q];
		}
		cameraPose[p*4+3] = _mt[p];
	}

	cvReleaseMat(&intric);

	delete _nepnp;
	return _rerror;
}
