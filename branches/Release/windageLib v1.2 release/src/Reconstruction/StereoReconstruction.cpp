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

#include "Reconstruction/StereoReconstruction.h"
using namespace windage;
using namespace windage::Reconstruction;

#include <cv.h>

int ERANSACUpdateNumIters(double p, double ep, int model_points, int max_iters)
{
	double num, denom;
	p = MAX(p, 0.);
    p = MIN(p, 1.);
    ep = MAX(ep, 0.);
    ep = MIN(ep, 1.);

    // avoid inf's & nan's
    num = MAX(1. - p, DBL_MIN);
    denom = 1. - pow(1. - ep,model_points);
	num = log(num);
    denom = log(denom);
    
	int result = denom >= 0 || -num >= max_iters*(-denom) ? max_iters : cvRound(num/denom);
	return result;
}

void StereoReconstruction::CalculateNormalizedPoint()
{
	windage::Matrix3 intrinsic;
	CvMat* intrinsicMat = this->initialCameraParameter->GetIntrinsicMatrix();

	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			intrinsic.m[y][x] = CV_MAT_ELEM((*intrinsicMat), double, y, x);
		}
	}

	intrinsic = intrinsic.Inverse();
	
	int n = (int)this->matchedPoint1->size();
	for(int i=0; i<n; i++)
	{
		windage::Vector3 point1 = (*this->matchedPoint1)[i].GetPoint();
		windage::Vector3 point2 = (*this->matchedPoint2)[i].GetPoint();

		windage::Vector3 normalizedPoint1 = intrinsic * point1;
		windage::Vector3 normalizedPoint2 = intrinsic * point2;

		this->normalizedMatchedPoint1.push_back(normalizedPoint1);
		this->normalizedMatchedPoint2.push_back(normalizedPoint2);
	}

	reconstructionPoints.resize(n);
}

bool StereoReconstruction::CalibratedTriangulation(CvMat *matR, CvMat *matT, CvMat *ptL, CvMat *ptR, CvMat *pt3D)
{
	if(ptL->cols <= 0 || ptR->cols <= 0) return false;

	/** how many points? */
	int ncount = ptL->cols; 

	/** solve . MX = 0 */
	CvMat *_M       = cvCreateMat( 3*ncount, ncount + 1, CV_64F);
	cvZero(_M);

	CvMat *_xb  = cvCreateMat(3, 3, CV_64F);
	CvMat *_t1  = cvCreateMat(3, 3, CV_64F);
	CvMat *_tc1 = cvCreateMat(3, 1, CV_64F);
	CvMat *_tc2 = cvCreateMat(3, 1, CV_64F);
	CvMat *_ptL = cvCreateMat(3, 1, CV_64F);

	for(int i=0; i<ncount; i++)
	{
		cvZero(_xb); 
		cvZero(_t1); 
		cvZero(_tc1); 
		cvZero(_tc2); 

		cvmSet(_xb, 0, 1, -cvmGet(ptR, 2, i));
		cvmSet(_xb, 0, 2,  cvmGet(ptR, 1, i));
		cvmSet(_xb, 1, 0,  cvmGet(ptR, 2, i));
		cvmSet(_xb, 1, 2, -cvmGet(ptR, 0, i));
		cvmSet(_xb, 2, 0, -cvmGet(ptR, 1, i));
		cvmSet(_xb, 2, 1,  cvmGet(ptR, 0, i));

		cvMatMul(_xb, matR, _t1); 
		cvmSet(_ptL, 0, 0, cvmGet(ptL, 0, i));
		cvmSet(_ptL, 1, 0, cvmGet(ptL, 1, i));
		cvmSet(_ptL, 2, 0, cvmGet(ptL, 2, i));
		cvMatMul(_t1, _ptL, _tc1); 
	
		cvMatMul(_xb, matT, _tc2); 

		cvmSet(_M, 0 + 3*i, i, cvmGet(_tc1, 0, 0));
		cvmSet(_M, 1 + 3*i, i, cvmGet(_tc1, 1, 0));
		cvmSet(_M, 2 + 3*i, i, cvmGet(_tc1, 2, 0));

		cvmSet(_M, 0 + 3*i, ncount, cvmGet(_tc2, 0, 0));
		cvmSet(_M, 1 + 3*i, ncount, cvmGet(_tc2, 1, 0));
		cvmSet(_M, 2 + 3*i, ncount, cvmGet(_tc2, 2, 0));
	}	


	CvMat *VT, *D;
	D = cvCreateMat(3*ncount, ncount+1, CV_64F);
	VT= cvCreateMat(ncount+1, ncount+1, CV_64F);

	cvSVD(_M, D, NULL, VT, CV_SVD_V_T);

	/** Retreive scale value */ 
	for(int i=0; i<ncount; i++)
	{
		cvmSet(pt3D, 0, i, cvmGet(ptL, 0, i) * cvmGet(VT, ncount, i)/cvmGet(VT, ncount, ncount));
		cvmSet(pt3D, 1, i, cvmGet(ptL, 1, i) * cvmGet(VT, ncount, i)/cvmGet(VT, ncount, ncount));
		cvmSet(pt3D, 2, i, cvmGet(ptL, 2, i) * cvmGet(VT, ncount, i)/cvmGet(VT, ncount, ncount));
		cvmSet(pt3D, 3, i, 1.0);
	}

	cvReleaseMat(&D);
	cvReleaseMat(&VT);

	cvReleaseMat(&_ptL);
	cvReleaseMat(&_xb);
	cvReleaseMat(&_t1);
	cvReleaseMat(&_tc1);
	cvReleaseMat(&_tc2);
	cvReleaseMat(&_M);

	return true;
}

bool StereoReconstruction::DecomposeEMatrix(CvMat *EMat)
{
	bool _failed = false;
	
	CvMat* localExtrinsic = cvCreateMat(4, 4, CV_64F);
	cvZero(localExtrinsic);
	CV_MAT_ELEM((*localExtrinsic), double, 3, 3) = 1.0;

	CvMat *_U, *_D, *_VT;
	_U  = cvCreateMat(3, 3, CV_64F);
	_D  = cvCreateMat(3, 3, CV_64F);
	_VT = cvCreateMat(3, 3, CV_64F);
	
	cvSVD(EMat, _D, _U, _VT, CV_SVD_V_T);
		
	/** enforce _D(0,0) == _D(1,1) */
	double _m = (cvmGet(_D, 0, 0) + cvmGet(_D, 1, 1))/2.0;
	cvmSet(_D, 0, 0, _m);
	cvmSet(_D, 1, 1, _m);
	cvmSet(_D, 2, 2,  0);
	cvMatMul(_U, _D, _U);
	cvMatMul(_U, _VT, EMat);
	cvSVD(EMat, _D, _U, _VT, CV_SVD_V_T);

	/** generate four possible solution */ 
	CvMat *_R[2], *_t[2], *_W;
	for(int i=0; i<2; i++) 
	{
		_R[i]  = cvCreateMat(3, 3, CV_64F);	
		_t[i]  = cvCreateMat(3, 1, CV_64F);	
	}

	for(int i=0; i<3; i++) cvmSet(_t[0], i, 0,        cvmGet(_U, i, 2));
	for(int i=0; i<3; i++) cvmSet(_t[1], i, 0, -1.0 * cvmGet(_U, i, 2));
	
	_W = cvCreateMat(3, 3, CV_64F);
	cvZero(_W);
	cvmSet(_W, 0, 1,  -1.0); cvmSet(_W, 1, 0, 1.0); cvmSet(_W, 2, 2, 1.0);
	cvMatMul(_U, _W, _R[0]);
	cvMatMul(_R[0], _VT, _R[0]);

	cvZero(_W);
	cvmSet(_W, 0, 1,   1.0); cvmSet(_W, 1, 0, -1.0); cvmSet(_W, 2, 2, 1.0);
	cvMatMul(_U, _W, _R[1]);
	cvMatMul(_R[1], _VT, _R[1]);

	cvReleaseMat(&_U);
	cvReleaseMat(&_D);
	cvReleaseMat(&_VT);
	cvReleaseMat(&_W);

	double det0 = cvDet(_R[0]);
	double det1 = cvDet(_R[1]);
	double detk = cvDet(this->initialCameraParameter->GetIntrinsicMatrix());
	if(det0*detk < 0) cvScale(_R[0], _R[0], -1.);
	if(det1*detk < 0) cvScale(_R[1], _R[1], -1.);

	/** generate one good solution by checking the point Z value */ 
	CvMat *testPt  = cvCreateMat(4, 1, CV_64F);
	CvMat *testLPt = cvCreateMat(3, 1, CV_64F);
	CvMat *testRPt = cvCreateMat(3, 1, CV_64F);
	CvMat *test2DPt= cvCreateMat(3, 1, CV_64F);
	for(int i=0; i<3; i++) 
	{
		cvmSet(testLPt, i, 0, this->normalizedMatchedPoint1[0].v[i]);
		cvmSet(testRPt, i, 0, this->normalizedMatchedPoint2[0].v[i]);
	}

	int ir = -1, it = -1; 
	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
		{
			for(int y=0; y<3; y++)
			{
				for(int x=0; x<3; x++)
				{
					double valeue = CV_MAT_ELEM((*_R[i]), double, y, x);
					CV_MAT_ELEM((*localExtrinsic), double, y, x) = valeue;
				}
				double valeue = CV_MAT_ELEM((*_t[j]), double, y, 0);
				CV_MAT_ELEM((*localExtrinsic), double, y, 3) = valeue;
			}
			this->localCameraParameter->SetExtrinsicMatrix(localExtrinsic);

			CalibratedTriangulation(_R[i], _t[j], testLPt, testRPt, testPt);
			
			double lv, rv;

			this->initialCameraParameter->ConvertWorld2Image(test2DPt, testPt);
			lv = cvmGet(test2DPt, 2, 0);
			this->localCameraParameter->ConvertWorld2Image(test2DPt, testPt);
			rv = cvmGet(test2DPt, 2, 0);

			if(cvmGet(testPt, 2, 0)/cvmGet(testPt, 3, 0) > 0 && lv > 0 && rv > 0) // Z value and depth should be positive
			{
				for(int p=0; p<3; p++) 
				{
					cvmSet(testLPt, p, 0, this->normalizedMatchedPoint1[1].v[p]);
					cvmSet(testRPt, p, 0, this->normalizedMatchedPoint2[1].v[p]);
				}
				
				CalibratedTriangulation(_R[i], _t[j], testLPt, testRPt, testPt);
				
				this->initialCameraParameter->ConvertWorld2Image(test2DPt, testPt);
				lv = cvmGet(test2DPt, 2, 0);
				this->localCameraParameter->ConvertWorld2Image(test2DPt, testPt);
				rv = cvmGet(test2DPt, 2, 0);

				if(cvmGet(testPt, 2, 0)/cvmGet(testPt, 3, 0) > 0 && lv > 0 && rv > 0) // Z value and depth should be positive 
				{
					ir = i, it = j; 
				}
			}
		}
	}

	cvReleaseMat(&testPt);
	cvReleaseMat(&testLPt);
	cvReleaseMat(&testRPt);
	cvReleaseMat(&test2DPt);

	if(ir >= 0 && it >= 0)
	{
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				double valeue = CV_MAT_ELEM((*_R[ir]), double, y, x);
				CV_MAT_ELEM((*localExtrinsic), double, y, x) = valeue;
			}
			double valeue = CV_MAT_ELEM((*_t[it]), double, y, 0);
			CV_MAT_ELEM((*localExtrinsic), double, y, 3) = valeue;
		}
		this->localCameraParameter->SetExtrinsicMatrix(localExtrinsic);
		_failed = false;
	}
	else
	{
		_failed = true;
	}

	for(int i=0; i<2; i++) 
	{
		cvReleaseMat(&_R[i]);
		cvReleaseMat(&_t[i]);
	}
	cvReleaseMat(&localExtrinsic);

	return !_failed;
}

int StereoReconstruction::ReconstructAll(CvMat *matE)
{
	int _bad_points = 0;

	CvMat *testLPt = cvCreateMat(3, 1, CV_64F);
	CvMat *testRPt = cvCreateMat(3, 1, CV_64F);
	CvMat *testPt  = cvCreateMat(4, 1, CV_64F);

	CvMat* _R = cvCreateMat(3, 3, CV_64F);
	CvMat* _t = cvCreateMat(3, 1, CV_64F);

	int n = (int)this->normalizedMatchedPoint1.size();

	for(int i=0; i<n; i++)
	{
		for(int k=0; k<3; k++) 
		{
			cvmSet(testLPt, k, 0, this->normalizedMatchedPoint1[i].v[k]);
			cvmSet(testRPt, k, 0, this->normalizedMatchedPoint2[i].v[k]);
		}

		CvMat* localExtrinsic = this->localCameraParameter->GetExtrinsicMatrix();
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				double valeue = CV_MAT_ELEM((*localExtrinsic), double, y, x);
				CV_MAT_ELEM((*_R), double, y, x) = valeue;
			}
			double valeue = CV_MAT_ELEM((*localExtrinsic), double, y, 3);
			CV_MAT_ELEM((*_t), double, y, 0) = valeue;
		}

		CalibratedTriangulation(_R, _t, testLPt, testRPt, testPt);

		double ww = 1.0 / cvmGet(testPt, 3, 0);
		windage::Vector4 tempPoint;
		for(int j=0; j<4; j++)
			tempPoint.v[j] = cvmGet(testPt, j, 0) * ww;
		reconstructionPoints[i].SetPoint(tempPoint);

		this->initialCameraParameter->ConvertWorld2Image(testLPt, testPt);
		this->localCameraParameter->ConvertWorld2Image(testRPt, testPt);

		if(this->reconstructionPoints[i].GetPoint().z < 0 || testRPt->data.db[2] < 0 || testLPt->data.db[2] < 0)
		{
			this->reconstructionPoints[i].SetOutlier(true);
			//TRACE("%f %f %f\n", cvmGet(m_pt3D, 2, i), testRPt->data.db[2], testLPt->data.db[2]);
			_bad_points++; /** check z-value */
		}
	}

	cvReleaseMat(&_R);
	cvReleaseMat(&_t);

	cvReleaseMat(&testLPt);
	cvReleaseMat(&testRPt);
	cvReleaseMat(&testPt);

	return _bad_points;
}


int StereoReconstruction::CountInliers(double thresh, double *err)
{
	int inliers = 0;
	CvMat *tpt1, *tpt2;	
	tpt1 = cvCreateMat(3, 1, CV_64F);
	tpt2 = cvCreateMat(3, 1, CV_64F);
	CvMat* tempPt = cvCreateMat(4, 1, CV_64F);

	double le, re, error = 0.0;
	int n = (int)this->reconstructionPoints.size();
	for(int i=0; i<n; i++)
	{
		CV_MAT_ELEM((*tempPt), double, 0, 0) = this->reconstructionPoints[i].GetPoint().x;
		CV_MAT_ELEM((*tempPt), double, 1, 0) = this->reconstructionPoints[i].GetPoint().y;
		CV_MAT_ELEM((*tempPt), double, 2, 0) = this->reconstructionPoints[i].GetPoint().z;
		CV_MAT_ELEM((*tempPt), double, 3, 0) = this->reconstructionPoints[i].GetPoint().w;

		windage::Vector4 wPT = this->reconstructionPoints[i].GetPoint();
		CvPoint rPT1 = this->initialCameraParameter->ConvertWorld2Image(wPT.x, wPT.y, wPT.z);
		CV_MAT_ELEM((*tpt1), double, 0, 0) = rPT1.x;
		CV_MAT_ELEM((*tpt1), double, 1, 0) = rPT1.y;
		CV_MAT_ELEM((*tpt1), double, 2, 0) = 1.0;

		CvPoint rPT2 = this->localCameraParameter->ConvertWorld2Image(wPT.x, wPT.y, wPT.z);
		CV_MAT_ELEM((*tpt2), double, 0, 0) = rPT2.x;
		CV_MAT_ELEM((*tpt2), double, 1, 0) = rPT2.y;
		CV_MAT_ELEM((*tpt2), double, 2, 0) = 1.0;

		le = ((*this->matchedPoint1)[i].GetPoint().x - cvmGet(tpt1, 0, 0))*((*this->matchedPoint1)[i].GetPoint().x - cvmGet(tpt1, 0, 0)) + 
			 ((*this->matchedPoint1)[i].GetPoint().y - cvmGet(tpt1, 1, 0))*((*this->matchedPoint1)[i].GetPoint().y - cvmGet(tpt1, 1, 0));

		re = ((*this->matchedPoint2)[i].GetPoint().x - cvmGet(tpt2, 0, 0))*((*this->matchedPoint2)[i].GetPoint().x - cvmGet(tpt2, 0, 0)) + 
			 ((*this->matchedPoint2)[i].GetPoint().y - cvmGet(tpt2, 1, 0))*((*this->matchedPoint2)[i].GetPoint().y - cvmGet(tpt2, 1, 0));

		double localError =  (cvSqrt(le) + cvSqrt(re)) / 2.0;
		if(localError < thresh)
		{
			this->reconstructionPoints[i].SetOutlier(false);
			error += localError;
			inliers++;
		}
		else
		{
			this->reconstructionPoints[i].SetOutlier(true);
		}
	}

	cvReleaseMat(&tempPt);
	cvReleaseMat(&tpt1);
	cvReleaseMat(&tpt2);

	if(inliers > 0)
	{
		*err = error/inliers;
	}
	else
	{
		*err = -1;
	}

	return inliers;
}

bool StereoReconstruction::ComputeEssentialMatrix8Points(CvMat *pt1, CvMat *pt2, CvMat *EMat)
{
	bool bsuccess = true; 

	CvMat *Q, *D, *V;
	Q  = cvCreateMat(9, 9, CV_64F);
	D  = cvCreateMat(9, 9, CV_64F);
	V  = cvCreateMat(9, 9, CV_64F);

	for(int i=0; i<8; i++)
	{
		double _pl1 = cvmGet(pt1, 0, i);
		double _pl2 = cvmGet(pt1, 1, i);
		double _pl3 = cvmGet(pt1, 2, i);
		double _pr1 = cvmGet(pt2, 0, i);
		double _pr2 = cvmGet(pt2, 1, i);
		double _pr3 = cvmGet(pt2, 2, i);

		cvmSet(Q, i, 0, _pl1*_pr1);
		cvmSet(Q, i, 1, _pl2*_pr1);
		cvmSet(Q, i, 2, _pl3*_pr1);
		cvmSet(Q, i, 3, _pl1*_pr2);
		cvmSet(Q, i, 4, _pl2*_pr2);
		cvmSet(Q, i, 5, _pl3*_pr2);
		cvmSet(Q, i, 6, _pl1*_pr3);
		cvmSet(Q, i, 7, _pl2*_pr3);
		cvmSet(Q, i, 8, _pl3*_pr3);
	}

	//scvPrintMat(Q);

	cvSVD(Q, D, NULL, V, CV_SVD_V_T);

	for(int i=0; i<9; i++) 
		EMat->data.db[i] =  cvmGet(V, 8, i);;
	 
	cvScale(EMat, EMat, 1.0/cvNorm(EMat));

	cvReleaseMat(&Q);
	cvReleaseMat(&D);
	cvReleaseMat(&V);

	return true;
}

bool StereoReconstruction::ComputeEssentialMatrixRANSAC(double* error)
{
	// preparation
	int n = (int)this->normalizedMatchedPoint1.size();
	const int SAMPLE_SIZE = 8;
	if(n < SAMPLE_SIZE)
	{
		(*error) = -1.0;
		return false;
	}

	int max_iter = 0, ci = 0;
	int max_random_iters = 20;
	int total_num = n;
	
	CvMat *EMat, *pt1, *pt2;
	EMat  = cvCreateMat(3, 3, CV_64F);
	pt1   = cvCreateMat(3, SAMPLE_SIZE, CV_64F);
	pt2   = cvCreateMat(3, SAMPLE_SIZE, CV_64F);

	CvRNG rng = cvRNG(cvGetTickCount());
	int pre_inlier = -1, num_inlier;
	double pre_error = 10000, small_err = 0.0;
		
	max_iter = this->maxIteration;
	int *idx = new int[SAMPLE_SIZE];

	double err = 0.0;
	while(ci < max_iter)
	{
		/** select random number */
		int snum = 0;
        bool bIn;
        for(int i = 0; i < SAMPLE_SIZE; i++ )
        {
			for(int k = 0; k <max_random_iters; k++ )
			{
				/** randomly select one */
				idx[i] = cvRandInt(&rng) % total_num;

				bIn = false;
				/** is the picked one already chosen? */
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
        } /** done */

		for(int i=0; i<SAMPLE_SIZE; i++)
		{
			cvmSet(pt1, 0, i, this->normalizedMatchedPoint1[idx[i]].x);
			cvmSet(pt1, 1, i, this->normalizedMatchedPoint1[idx[i]].y);
			cvmSet(pt1, 2, i, this->normalizedMatchedPoint1[idx[i]].z);

			cvmSet(pt2, 0, i, this->normalizedMatchedPoint2[idx[i]].x);
			cvmSet(pt2, 1, i, this->normalizedMatchedPoint2[idx[i]].y);
			cvmSet(pt2, 2, i, this->normalizedMatchedPoint2[idx[i]].z);
		}

		/** compute E matrix */
		bool es;
		es = ComputeEssentialMatrix8Points(pt1, pt2, EMat);

		if(es)
		{
			/** decompose E into R, t */
			if(DecomposeEMatrix(EMat))
			{
				/** reconstruct all pt */
				int _bad_pt = ReconstructAll(EMat);

				/** count inliers */
				err = 0.0;
				num_inlier = CountInliers(this->reprojectionError, &err);

				if(num_inlier > _bad_pt * 10) /** check decomposition is good or not */
				{
					if(num_inlier > pre_inlier)
					{
						cvCopy(EMat, essentialMatrix); 
						pre_inlier = num_inlier;
						if(error)
							*error = err;

						if(this->confidence > 0)
							max_iter = ERANSACUpdateNumIters(this->confidence, (double)(n - num_inlier)/(double)n, SAMPLE_SIZE, max_iter);
					}
				}

			}
		}
		ci++;
	}

	delete [] idx;

	/** final check */
	DecomposeEMatrix(essentialMatrix);
	int bnum = ReconstructAll(essentialMatrix);
	this->inlierCount = CountInliers(this->reprojectionError, &err);

	cvReleaseMat(&pt1);
	cvReleaseMat(&pt2);
	cvReleaseMat(&EMat);

	if(pre_inlier == 0 || *error > 5.0)
	{
		return false;
	}

	return true;
}
