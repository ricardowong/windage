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
	if(k>5)
	{
		_ret = RunEpnpRANSAC(this->matchedPoints, this->cameraPose);
	}
	return _ret;
}

double FindEpnpPoseEstimation::RunEpnpRANSAC(std::vector<Matched3DPoint>* matchedPoints, double* cameraPose)
{
	double _rerror = -1.0;
	int numpt = (int)matchedPoints->size();
	if(numpt < 5) return -1;

	CvMat *intric = cvCreateMat(3, 3, CV_64F);
	cvmSetZero(intric);
	cvmSet(intric, 0, 0, this->intrinsic[0]);
	cvmSet(intric, 1, 1, this->intrinsic[1]);
	cvmSet(intric, 0, 2, this->intrinsic[2]);
	cvmSet(intric, 1, 2, this->intrinsic[3]);
	cvmSet(intric, 2, 2, 1.0);

	// Pose estimation using PnP alogrithm from EPFL
	epnp *_epnp = new epnp;
	_epnp->set_internal_parameters(this->intrinsic[2], this->intrinsic[3], this->intrinsic[0], this->intrinsic[1]);

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
	_nepnp->set_internal_parameters(this->intrinsic[2], this->intrinsic[3], this->intrinsic[0], this->intrinsic[1]);
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

/** borrowed code from kcvLib */
/*
double epnpS(CSIFTObject *sobj, CvMat *outRT)
{
	std::vector<CvMat*> pt3D;
	std::vector<CvMat*> pt2D;

	// fill
	int k = 0;
	double _ret = -1;
	for(int i=0; i<(int)sobj->m_listSIFTfeature.size(); i++)
	{
		int _keyi = sobj->m_listSIFTfeature[i].m_matchinfo;
		if(_keyi > 0)
		{
			// 3D pt
			pt3D.push_back((*m_ptr3DPoint)[_keyi]->Get3DPoint());

			// 2D pt
			CvMat *cpt = cvCreateMat(3, 1, CV_64F);
			cvmSet(cpt, 0, 0, sobj->m_listSIFTfeature[i].m_x);
			cvmSet(cpt, 1, 0, sobj->m_listSIFTfeature[i].m_y);
			cvmSet(cpt, 2, 0, 1.0);
			
			pt2D.push_back(cpt);

			k++;
		}
	}

	if(k>5)
	{
		CvMat *status = cvCreateMat(1, k, CV_64F);

		_ret = epnpRANSAC(&pt3D, &pt2D, outRT, status);

		k = 0;
		for(int i=0; i<(int)sobj->m_listSIFTfeature.size(); i++)
		{
			int _keyi = sobj->m_listSIFTfeature[i].m_matchinfo;
			if(_keyi > 0)
			{
				if(status->data.db[k] > 0)
				{
					//sobj->m_listSIFTfeature[i].m_matchinfo
				}
				else
				{
					sobj->m_listSIFTfeature[i].m_matchinfo = -1; // we keep only inliers
				}

				k++;
			}
		}

		cvReleaseMat(&status);
	}

	for(int i=0; i<(int)pt2D.size(); i++)
	{
		cvReleaseMat(&pt2D[i]);
	}

	return _ret;
}

double epnpRANSAC(std::vector<CvMat*> *pt3D, std::vector<CvMat*> *pt2D, CvMat *outRT, CvMat *status)
{
	double _rerror = -1.0;
	

	if((*pt3D).size() < 5) return -1;

	// Pose estimation using PnP alogrithm from EPFL
	epnp *_epnp = new epnp;
	_epnp->set_internal_parameters(m_matIntrinsic->data.db[2], m_matIntrinsic->data.db[5], m_matIntrinsic->data.db[0], m_matIntrinsic->data.db[4]);

	int sample_size = 5, pre_inlier = -1, num_inliers = 0;
	int max_random_iters = 10, ci = 0;
	const unsigned rng_seed = 0xffffffff;
	CvRNG rng = cvRNG(rng_seed);
	double pre_error = 10000, small_err = 10000.0, _mR[3][3], _mt[3];

	int numpt = (*pt3D).size();
	
	

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
			_X = cvmGet((*pt3D)[idx[i]], 0, 0);
			_Y = cvmGet((*pt3D)[idx[i]], 1, 0);
			_Z = cvmGet((*pt3D)[idx[i]], 2, 0);
			_u = cvmGet((*pt2D)[idx[i]], 0, 0);
			_v = cvmGet((*pt2D)[idx[i]], 1, 0);

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
		cvMatMul(m_matIntrinsic, Proj, Proj);

		double x, y, err;
		CvMat *p2d = cvCreateMat(3, 1, CV_64F);
		for(int i=0; i<(int)(*pt3D).size(); i++)
		{
			cvMatMul(Proj, (*pt3D)[i], p2d);
			x = p2d->data.db[0]/p2d->data.db[2];
			y = p2d->data.db[1]/p2d->data.db[2];
			err = ((*pt2D)[i]->data.db[0] - x)*((*pt2D)[i]->data.db[0] - x) + ((*pt2D)[i]->data.db[1] - y)*((*pt2D)[i]->data.db[1] - y);
				
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
	_nepnp->set_internal_parameters(m_matIntrinsic->data.db[2], m_matIntrinsic->data.db[5], m_matIntrinsic->data.db[0], m_matIntrinsic->data.db[4]);
	_nepnp->set_maximum_number_of_correspondences(pre_inlier);
	_nepnp->reset_correspondences();

	for(int i=0; i<numpt; i++)
	{
		if(bStatus->data.db[i] > 0)
		{
			double _X, _Y, _Z, _u, _v;
			_X = cvmGet((*pt3D)[i], 0, 0);
			_Y = cvmGet((*pt3D)[i], 1, 0);
			_Z = cvmGet((*pt3D)[i], 2, 0);
			_u = cvmGet((*pt2D)[i], 0, 0);
			_v = cvmGet((*pt2D)[i], 1, 0);

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
	cvMatMul(m_matIntrinsic, Proj, Proj);

	double x, y, err;
	CvMat *p2d = cvCreateMat(3, 1, CV_64F);
	for(int i=0; i<(int)(*pt3D).size(); i++)
	{
		cvMatMul(Proj, (*pt3D)[i], p2d);
		x = p2d->data.db[0]/p2d->data.db[2];
		y = p2d->data.db[1]/p2d->data.db[2];
		err = ((*pt2D)[i]->data.db[0] - x)*((*pt2D)[i]->data.db[0] - x) + ((*pt2D)[i]->data.db[1] - y)*((*pt2D)[i]->data.db[1] - y);

		if(cvSqrt(err) < 2.0)
		{
			num_inliers++;
			status->data.db[i] = 1.0;
		}
		else
		{
			status->data.db[i] = 0.0;
		}
	}
	cvReleaseMat(&Proj);
	cvReleaseMat(&p2d);

	cvReleaseMat(&tStatus);
	cvReleaseMat(&bStatus);
	

	if(num_inliers < 5) _rerror = -1.0;

	for(int p=0; p<3; p++)
	{
		for(int q=0; q<3; q++)
		{
			cvmSet(outRT, p, q, _mR[p][q]);
		}
		cvmSet(outRT, p, 3, _mt[p]);
	}

	delete _nepnp;

	return _rerror;
}
//*/