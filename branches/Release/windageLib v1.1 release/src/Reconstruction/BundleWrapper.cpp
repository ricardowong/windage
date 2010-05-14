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
#include <stdio.h>

#include "Reconstruction/BundleWrapper.h"
using namespace windage;
using namespace windage::Reconstruction;

struct globs_{
	double *intrcalib; /* the 5 intrinsic calibration parameters in the order [fu, u0, v0, ar, skew],
					   * where ar is the aspect ratio fv/fu.
					   * Used only when calibration is fixed for all cameras;
					   * otherwise, it is null and the intrinsic parameters are
					   * included in the set of motion parameters for each camera
					   */
	int nccalib; /* number of calibration parameters that must be kept constant.
				 * 0: all paremeters are free 
				 * 1: skew is fixed to its initial value, all other parameters vary (i.e. fu, u0, v0, ar) 
				 * 2: skew and aspect ratio are fixed to their initial values, all other parameters vary (i.e. fu, u0, v0)
				 * 3: meaningless
				 * 4: skew, aspect ratio and principal point are fixed to their initial values, only the focal length varies (i.e. fu)
				 * >=5: meaningless
				 * Used only when calibration varies among cameras
				 */

	int cnp, pnp, mnp; /* dimensions */

	double *ptparams;  /* needed only when bundle adjusting for camera parameters only */
	double *camparams; /* needed only when bundle adjusting for structure parameters only */
} globs;

BundleWrapper::BundleWrapper(void)
{
	m_cnp = 6; /** dim. rt */
	m_pnp = 3; /** dim . point */
	m_mnp = 2; /** measurement */
	m_itmax = 100; /** max iteration */

	/* set up globs structure */
	globs.cnp = m_cnp; 
	globs.pnp = m_pnp; 
	globs.mnp = m_mnp;
}
BundleWrapper::~BundleWrapper(void)
{
}

void BundleWrapper::SetParameters(CvMat *intrinsic, CvMat *pt3D, CvMat **pt2D, CvMat **RT, int nImage, int nPTs)
{
	m_intrinsicsba[0] = intrinsic->data.db[0];
	m_intrinsicsba[1] = intrinsic->data.db[2];
	m_intrinsicsba[2] = intrinsic->data.db[5];
	m_intrinsicsba[3] = intrinsic->data.db[4]/intrinsic->data.db[0];
	m_intrinsicsba[4] = intrinsic->data.db[1];
	globs.intrcalib = m_intrinsicsba;
	globs.ptparams = NULL;
	globs.camparams = NULL;

	/* call sparse LM routine */
	m_opts[0]=SBA_INIT_MU; m_opts[1]=SBA_STOP_THRESH; m_opts[2]=SBA_STOP_THRESH;
	m_opts[3]=SBA_STOP_THRESH;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	m_opts[4]=0.0;

	m_pt3D   = pt3D;
	m_pt2D   = pt2D;
	m_RT     = RT;
	m_nPT    = nPTs;
	m_nImage = nImage;
}

bool BundleWrapper::Run()
{
	bool _ret = true;

	int nconstframes = 1;
	int verbose = 1;

	char *vmask = (char *)malloc(m_nPT * m_nImage * sizeof(char));
	double *motstruct = (double *)malloc((m_nImage * m_cnp + m_nPT * m_pnp)*sizeof(double));
	double *imgpts = (double *)malloc(m_nPT * m_nImage * m_mnp * sizeof(double));
	double *covimgpts = NULL; /** no covariance */
  
	printf("Euclidean Sparse Bundle Adjustment ... \n");

	/** 
	 * fill matricess 
	*/

	/** RT */
	int pindex = 0;
	for(int i=0; i<m_nImage; i++)
	{
		CvMat *R = cvCreateMat(3, 3, CV_64F);
		double q[4], v[3], t[3];
		for(int p=0; p<3; p++)
		{
			for(int h=0; h<3; h++)
			{
				cvmSet(R, p, h, cvmGet(m_RT[i], p, h));
			}
			t[p] = cvmGet(m_RT[i], p, 3);
		}

		Matrix2Quaternion(R, &q[0]);
		quat2vec(q, 0, v, 0);

		for(int j=0; j<3; j++) motstruct[j + pindex] = v[j];
		for(int j=0; j<3; j++) motstruct[j+3 + pindex] = t[j];
		pindex += 6;

		cvReleaseMat(&R);
	}

	
	/**	3D points */
	for(int i=0; i<m_nPT; i++)
	{
		motstruct[pindex+0] = cvmGet(m_pt3D, 0, i);
		motstruct[pindex+1] = cvmGet(m_pt3D, 1, i);
		motstruct[pindex+2] = cvmGet(m_pt3D, 2, i);
		pindex += 3;
	}

	/** 2D points */
	pindex = 0;

	for(int p=0; p<m_nPT; p++)
	{
		for(int i=0; i<m_nImage; i++)
		{
			if(cvmGet(m_pt2D[i], 0, p) > 0 && cvmGet(m_pt2D[i], 1, p) > 0)
			{
				imgpts[pindex] = cvmGet(m_pt2D[i], 0, p);
				imgpts[pindex+1] = cvmGet(m_pt2D[i], 1, p);
				pindex += 2;
			}
		}
	}

	/**
	 * vmask
	*/
	pindex = 0;
	for(int p=0; p<m_nPT; p++)
	{
		for(int i=0; i<m_nImage; i++)
		{
			if(cvmGet(m_pt2D[i], 0, p) < 0 && cvmGet(m_pt2D[i], 1, p) < 0) vmask[pindex] = 0;
			else vmask[pindex] = 1;

			pindex++;
		}
	}


	int _iret = sba_motstr_levmar_x(m_nPT, m_nImage, nconstframes, vmask, motstruct, m_cnp, m_pnp, imgpts, covimgpts, m_mnp,
									img_projsRTS_x, img_projsRTS_jac_x, (void *)(&globs), m_itmax, verbose, m_opts, m_info);

	printf("Iteration #: %d \n", _iret);

	if(_iret >= 0) /** successful */
	{
		/** RT */
		pindex = 0;
		for(int i=0; i<m_nImage; i++)
		{
			double q[4], v[3], t[3];
			for(int j=0; j<3; j++) v[j] = motstruct[j + pindex];
			for(int j=0; j<3; j++) t[j] = motstruct[j + 3 + pindex];
			pindex += 6;

			CvMat *R = cvCreateMat(3, 3, CV_64F);
		
			vec2quat(v, 0, q, 0);
			Quaternion2Matrix(&q[0], R);
			
			for(int p=0; p<3; p++)
			{
				for(int h=0; h<3; h++)
				{
					cvmSet(m_RT[i], p, h, cvmGet(R, p, h));
				}
				cvmSet(m_RT[i], p, 3, t[p]);
			}

			cvReleaseMat(&R);
		}

		/**	3D points */
		for(int i=0; i<m_nPT; i++)
		{
			cvmSet(m_pt3D, 0, i, motstruct[pindex+0]);
			cvmSet(m_pt3D, 1, i, motstruct[pindex+1]);
			cvmSet(m_pt3D, 2, i, motstruct[pindex+2]);
			pindex += 3;
		}
	}

	

	free(vmask);
	free(motstruct);
	free(imgpts);

	return _ret;
}


void BundleWrapper::quat2vec(double *inp, int nin, double *outp, int nout)
{
double mag, sg;
register int i;

  /* intrinsics */
  if(nin==5+7) // are they present?
    for(i=0; i<5; ++i)
      outp[i]=inp[i];
  else
    i=0;

  /* rotation */
  /* normalize and ensure that the quaternion's scalar component is positive; if not,
   * negate the quaternion since two quaternions q and -q represent the same
   * rotation
   */
  mag=sqrt(inp[i]*inp[i] + inp[i+1]*inp[i+1] + inp[i+2]*inp[i+2] + inp[i+3]*inp[i+3]);
  sg=(inp[i]>=0.0)? 1.0 : -1.0;
  mag=sg/mag;
  outp[i]  =inp[i+1]*mag;
  outp[i+1]=inp[i+2]*mag;
  outp[i+2]=inp[i+3]*mag;
  i+=3;

  /* translation*/
  for( ; i<nout; ++i)
    outp[i]=inp[i+1];
}


void BundleWrapper::vec2quat(double *inp, int nin, double *outp, int nout)
{
double w;
register int i;

  /* intrinsics */
  if(nin==5+7-1) // are they present?
    for(i=0; i<5; ++i)
      outp[i]=inp[i];
  else
    i=0;

  /* rotation */
  /* recover the quaternion's scalar component from the vector one */
  w=sqrt(1.0 - (inp[i]*inp[i] + inp[i+1]*inp[i+1] + inp[i+2]*inp[i+2]));
  outp[i]  = w;
  outp[i+1]=inp[i];
  outp[i+2]=inp[i+1];
  outp[i+3]=inp[i+2];
  i+=4;

  /* translation */
  for( ; i<nout; ++i)
    outp[i]=inp[i-1];
}

void BundleWrapper::Quaternion2Matrix(double *q, CvMat *mat)
{

	double mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] /= mag;
	q[1] /= mag;
	q[2] /= mag;
	q[3] /= mag;

	//1 - 2*qy2 - 2*qz2  2*qx*qy - 2*qz*qw    2*qx*qz + 2*qy*qw 
	//2*qx*qy + 2*qz*qw  1 - 2*qx2 - 2*qz2    2*qy*qz - 2*qx*qw 
	//2*qx*qz - 2*qy*qw  2*qy*qz + 2*qx*qw    1 - 2*qx2 - 2*qy2 
	//0 2q[2] 2q[3]

	cvmSet(mat, 0, 0, 1 - 2*q[2]*q[2] - 2*q[3]*q[3]);
	cvmSet(mat, 0, 1, 2*q[1]*q[2] - 2*q[0]*q[3]);
	cvmSet(mat, 0, 2, 2*q[1]*q[3] + 2*q[0]*q[2]);
	
	cvmSet(mat, 1, 0, 2*q[1]*q[2] + 2*q[0]*q[3]);
	cvmSet(mat, 1, 1, 1 - 2*q[1]*q[1] - 2*q[3]*q[3]);
	cvmSet(mat, 1, 2, 2*q[2]*q[3] - 2*q[0]*q[1]);

	cvmSet(mat, 2, 0, 2*q[1]*q[3]-2*q[0]*q[2]);
	cvmSet(mat, 2, 1, 2*q[0]*q[1]+2*q[2]*q[3]);
	cvmSet(mat, 2, 2, 1-2*q[1]*q[1]-2*q[2]*q[2]);
}

//w, x, y, z 
void BundleWrapper::Matrix2Quaternion(CvMat *mat, double *q)
{
	double trace = (cvTrace(mat).val[0] + 1.0f);

	if(trace > 10E-7) 
	{
		double s = 0.5f/sqrt(trace);
		q[0] = 0.25f / s;
		q[1] = (cvmGet(mat, 2,1) - cvmGet(mat, 1,2)) * s;
		q[2] = (cvmGet(mat, 0,2) - cvmGet(mat, 2,0)) * s;
		q[3] = (cvmGet(mat, 1,0) - cvmGet(mat, 0,1) ) * s;
	} else 
	{
		if( cvmGet(mat, 0,0) > cvmGet(mat, 1,1) && cvmGet(mat, 0,0) > cvmGet(mat, 2,2) ) 
		{
			double s = 2.0f * sqrt( 1.0f + cvmGet(mat, 0,0) - cvmGet(mat, 1,1) - cvmGet(mat, 2,2));
			q[0] = (cvmGet(mat, 1, 2) - cvmGet(mat, 2, 1) ) / s;
			q[1] = 0.25f * s;
			q[2] = (cvmGet(mat, 0, 1) + cvmGet(mat, 1, 0) ) / s;
			q[3] = (cvmGet(mat, 0, 2) + cvmGet(mat, 2, 0) ) / s;
		} 
		else if(cvmGet(mat, 1,1) > cvmGet(mat, 2, 2)) 
		{
			double s = 2.0f * sqrt( 1.0f + cvmGet(mat, 1, 1) - cvmGet(mat, 0, 0) - cvmGet(mat, 2, 2));
			q[0] = (cvmGet(mat, 0, 2) - cvmGet(mat, 2, 0) ) / s;
			q[1] = (cvmGet(mat, 0, 1) + cvmGet(mat, 1, 0) ) / s;
			q[2] = 0.25f * s;
			q[3] = (cvmGet(mat, 1, 2) + cvmGet(mat, 2, 1) ) / s;
		} 
		else 
		{
			double s = 2.0f * sqrt( 1.0f + cvmGet(mat, 2, 2) - cvmGet(mat, 0,0) - cvmGet(mat, 1, 1) );
			q[0] = (cvmGet(mat, 0, 1) - cvmGet(mat, 1, 0) ) / s;
			q[1] = (cvmGet(mat, 0, 2) + cvmGet(mat, 2, 0) ) / s;
			q[2] = (cvmGet(mat, 1, 2) + cvmGet(mat, 2, 1) ) / s;
			q[3] = 0.25f * s;
		}
	}

	//double mag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	//for(int i=0;i<4;i++) q[i]/=mag;

}


/* FULL BUNDLE ADJUSTMENT, I.E. SIMULTANEOUS ESTIMATION OF CAMERA AND STRUCTURE PARAMETERS */

/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * hx the prediction of the measurements, i.e. the projections of 3D points in the m images. The measurements
 * are returned in the order (hx_11^T, .. hx_1m^T, ..., hx_n1^T, .. hx_nm^T)^T, where hx_ij is the predicted
 * projection of the i-th point on the j-th camera.
 * Notice that depending on idxij, some of the hx_ij might be missing
 *
 */
void BundleWrapper::img_projsRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata)
{
	register int i, j;
	int cnp, pnp, mnp;
	double *pa, *pb, *pqr, *pt, *ppt, *pmeas, *Kparms;
	//int n;
	int m, nnz;
	struct globs_ *gl;

	gl=(struct globs_ *)adata;
	cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
	Kparms=gl->intrcalib;

	//n=idxij->nr;
	m=idxij->nc;
	pa=p; pb=p+m*cnp;

	for(j=0; j<m; ++j){
		/* j-th camera parameters */
		pqr=pa+j*cnp;
		pt=pqr+3; // quaternion vector part has 3 elements

		nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

		for(i=0; i<nnz; ++i){
			ppt=pb + rcsubs[i]*pnp;
			pmeas=hx + idxij->val[rcidxs[i]]*mnp; // set pmeas to point to hx_ij

			calcImgProj(Kparms, pqr, pt, ppt, pmeas); // evaluate Q in pmeas
		}
	}
}


/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * jac the jacobian of the predicted measurements, i.e. the jacobian of the projections of 3D points in the m images.
 * The jacobian is returned in the order (A_11, ..., A_1m, ..., A_n1, ..., A_nm, B_11, ..., B_1m, ..., B_n1, ..., B_nm),
 * where A_ij=dx_ij/db_j and B_ij=dx_ij/db_i (see HZ).
 * Notice that depending on idxij, some of the A_ij, B_ij might be missing
 *
 */
void BundleWrapper::img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
{
	register int i, j;
	int cnp, pnp, mnp;
	double *pa, *pb, *pqr, *pt, *ppt, *pA, *pB, *Kparms;
	//int n;
	int m, nnz, Asz, Bsz, ABsz, idx;
	struct globs_ *gl;

	gl=(struct globs_ *)adata;
	cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
	Kparms=gl->intrcalib;

	//n=idxij->nr;
	m=idxij->nc;
	pa=p; pb=p+m*cnp;
	Asz=mnp*cnp; Bsz=mnp*pnp; ABsz=Asz+Bsz;

	for(j=0; j<m; ++j){
		/* j-th camera parameters */
		pqr=pa+j*cnp;
		pt=pqr+3; // quaternion vector part has 3 elements

		nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

		for(i=0; i<nnz; ++i){
			ppt=pb + rcsubs[i]*pnp;
			idx=idxij->val[rcidxs[i]];
			pA=jac + idx*ABsz; // set pA to point to A_ij
			pB=pA  + Asz; // set pB to point to B_ij

			calcImgProjJacRTS(Kparms, pqr, pt, ppt, (double (*)[6])pA, (double (*)[3])pB); // evaluate dQ/da, dQ/db in pA, pB
		}
	}
}
void BundleWrapper::calcImgProj(double a[5],double v[3],double t[3],double M[3],double n[2])
{
  double t1;
  double t11;
  double t13;
  double t14;
  double t15;
  double t17;
  double t2;
  double t21;
  double t26;
  double t3;
  double t31;
  double t42;
  double t5;
  double t50;
  double t53;
  double t6;
  double t8;
  double t9;
  {
    t1 = a[0];
    t2 = v[0];
    t3 = M[0];
    t5 = v[1];
    t6 = M[1];
    t8 = v[2];
    t9 = M[2];
    t11 = -t2*t3-t5*t6-t8*t9;
    t13 = t2*t2;
    t14 = t5*t5;
    t15 = t8*t8;
    t17 = sqrt(1.0-t13-t14-t15);
    t21 = t17*t3+t5*t9-t8*t6;
    t26 = t17*t6+t8*t3-t2*t9;
    t31 = t17*t9+t2*t6-t5*t3;
    t42 = -t11*t5+t17*t26-t2*t31+t21*t8+t[1];
    t50 = -t11*t8+t17*t31-t21*t5+t2*t26+t[2];
    t53 = 1/t50;
    n[0] = (t1*(-t11*t2+t17*t21-t26*t8+t31*t5+t[0])+a[4]*t42+a[1]*t50)*t53;
    n[1] = (t1*a[3]*t42+a[2]*t50)*t53;
    return;
  }
}

void BundleWrapper::calcImgProjJacKRTS(double a[5],double v[3],double t[3],double M[3], double jacmKRT[2][11],double jacmS[2][3])
{
  double t1;
  double t10;
  double t104;
  double t109;
  double t111;
  double t114;
  double t12;
  double t121;
  double t128;
  double t13;
  double t14;
  double t141;
  double t144;
  double t154;
  double t159;
  double t16;
  double t17;
  double t171;
  double t172;
  double t175;
  double t177;
  double t179;
  double t18;
  double t181;
  double t19;
  double t194;
  double t195;
  double t197;
  double t199;
  double t2;
  double t20;
  double t213;
  double t215;
  double t22;
  double t23;
  double t24;
  double t25;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t33;
  double t39;
  double t4;
  double t40;
  double t41;
  double t47;
  double t49;
  double t5;
  double t51;
  double t52;
  double t54;
  double t56;
  double t58;
  double t6;
  double t60;
  double t64;
  double t66;
  double t7;
  double t70;
  double t71;
  double t73;
  double t75;
  double t77;
  double t8;
  double t80;
  double t83;
  double t9;
  double t91;
  double t92;
  double t93;
  double t95;
  double t97;
  {
    t1 = v[0];
    t2 = M[0];
    t3 = t1*t2;
    t4 = v[1];
    t5 = M[1];
    t6 = t4*t5;
    t7 = v[2];
    t8 = M[2];
    t9 = t7*t8;
    t10 = -t3-t6-t9;
    t12 = t1*t1;
    t13 = t4*t4;
    t14 = t7*t7;
    t16 = sqrt(1.0-t12-t13-t14);
    t17 = t16*t2;
    t18 = t4*t8;
    t19 = t7*t5;
    t20 = t17+t18-t19;
    t22 = t16*t5;
    t23 = t7*t2;
    t24 = t1*t8;
    t25 = t22+t23-t24;
    t27 = t16*t8;
    t28 = t1*t5;
    t29 = t4*t2;
    t30 = t27+t28-t29;
    t33 = -t1*t10+t16*t20-t25*t7+t30*t4+t[0];
    t39 = -t10*t7+t16*t30-t20*t4+t25*t1+t[2];
    t40 = 1/t39;
    jacmKRT[0][0] = t33*t40;
    t41 = a[3];
    t47 = -t4*t10+t16*t25-t30*t1+t20*t7+t[1];
    jacmKRT[1][0] = t41*t47*t40;
    jacmKRT[0][1] = 1.0;
    jacmKRT[1][1] = 0.0;
    jacmKRT[0][2] = 0.0;
    jacmKRT[1][2] = 1.0;
    jacmKRT[0][3] = 0.0;
    t49 = a[0];
    jacmKRT[1][3] = t49*t47*t40;
    jacmKRT[0][4] = t47*t40;
    jacmKRT[1][4] = 0.0;
    t51 = 1/t16;
    t52 = t51*t20;
    t54 = t51*t5;
    t56 = -t54*t1-t8;
    t58 = t51*t8;
    t60 = -t58*t1+t5;
    t64 = a[4];
    t66 = t51*t25;
    t70 = t51*t2;
    t71 = t1*t7;
    t73 = 2.0*t29-t66*t1+t16*t56-t60*t1-t27-t28-t70*t71;
    t75 = a[1];
    t77 = t51*t30;
    t80 = t4*t1;
    t83 = 2.0*t23-t77*t1+t16*t60+t70*t80+t56*t1+t22-t24;
    t91 = t39*t39;
    t92 = 1/t91;
    t93 = (t49*t33+t64*t47+t75*t39)*t92;
    jacmKRT[0][5] = (t49*(t3+t6+t9-t52*t1-t56*t7+t60*t4)+t64*t73+t75*t83)*t40-
t93*t83;
    t95 = t49*t41;
    t97 = a[2];
    t104 = (t95*t47+t97*t39)*t92;
    jacmKRT[1][5] = (t95*t73+t97*t83)*t40-t104*t83;
    t109 = -t70*t4+t8;
    t111 = t4*t7;
    t114 = -t58*t4-t2;
    t121 = t6+t3+t9-t66*t4-t114*t1+t109*t7;
    t128 = 2.0*t19-t77*t4+t114*t16-t109*t4-t17-t18-t54*t80;
    jacmKRT[0][6] = (t49*(2.0*t28-t52*t4+t16*t109+t54*t111+t114*t4+t27-t29)+t64
*t121+t75*t128)*t40-t93*t128;
    jacmKRT[1][6] = (t95*t121+t97*t128)*t40-t104*t128;
    t141 = -t70*t7-t5;
    t144 = -t54*t7+t2;
    t154 = 2.0*t18-t66*t7+t144*t16+t58*t71+t141*t7+t17-t19;
    t159 = t9+t3+t6-t77*t7-t141*t4+t144*t1;
    jacmKRT[0][7] = (t49*(2.0*t24-t52*t7+t16*t141-t144*t7-t22-t23-t58*t111)+t64
*t154+t75*t159)*t40-t93*t159;
    jacmKRT[1][7] = (t95*t154+t97*t159)*t40-t104*t159;
    jacmKRT[0][8] = t49*t40;
    jacmKRT[1][8] = 0.0;
    jacmKRT[0][9] = t64*t40;
    jacmKRT[1][9] = t95*t40;
    jacmKRT[0][10] = t75*t40-t93;
    jacmKRT[1][10] = t97*t40-t104;
    t171 = 2.0*t13;
    t172 = 2.0*t14;
    t175 = t16*t7;
    t177 = 2.0*t80+2.0*t175;
    t179 = t16*t4;
    t181 = 2.0*t71-2.0*t179;
    jacmS[0][0] = (t49*(1.0-t171-t172)+t64*t177+t75*t181)*t40-t93*t181;
    jacmS[1][0] = (t95*t177+t97*t181)*t40-t104*t181;
    t194 = 2.0*t12;
    t195 = 1.0-t194-t172;
    t197 = t16*t1;
    t199 = 2.0*t111+2.0*t197;
    jacmS[0][1] = (t49*(2.0*t80-2.0*t175)+t64*t195+t75*t199)*t40-t93*t199;
    jacmS[1][1] = (t95*t195+t97*t199)*t40-t104*t199;
    t213 = 2.0*t111-2.0*t197;
    t215 = 1.0-t194-t171;
    jacmS[0][2] = (t49*(2.0*t71+2.0*t179)+t64*t213+t75*t215)*t40-t93*t215;
    jacmS[1][2] = (t95*t213+t97*t215)*t40-t104*t215;
    return;
  }
}

void BundleWrapper::calcImgProjJacKRT(double a[5],double v[3],double t[3],double M[3], double jacmKRT[2][11])
{
  double t1;
  double t10;
  double t104;
  double t109;
  double t111;
  double t114;
  double t12;
  double t121;
  double t128;
  double t13;
  double t14;
  double t141;
  double t144;
  double t154;
  double t159;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t22;
  double t23;
  double t24;
  double t25;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t33;
  double t39;
  double t4;
  double t40;
  double t41;
  double t47;
  double t49;
  double t5;
  double t51;
  double t52;
  double t54;
  double t56;
  double t58;
  double t6;
  double t60;
  double t64;
  double t66;
  double t7;
  double t70;
  double t71;
  double t73;
  double t75;
  double t77;
  double t8;
  double t80;
  double t83;
  double t9;
  double t91;
  double t92;
  double t93;
  double t95;
  double t97;
  {
    t1 = v[0];
    t2 = M[0];
    t3 = t1*t2;
    t4 = v[1];
    t5 = M[1];
    t6 = t4*t5;
    t7 = v[2];
    t8 = M[2];
    t9 = t7*t8;
    t10 = -t3-t6-t9;
    t12 = t1*t1;
    t13 = t4*t4;
    t14 = t7*t7;
    t16 = sqrt(1.0-t12-t13-t14);
    t17 = t16*t2;
    t18 = t4*t8;
    t19 = t7*t5;
    t20 = t17+t18-t19;
    t22 = t16*t5;
    t23 = t7*t2;
    t24 = t1*t8;
    t25 = t22+t23-t24;
    t27 = t16*t8;
    t28 = t1*t5;
    t29 = t4*t2;
    t30 = t27+t28-t29;
    t33 = -t1*t10+t16*t20-t25*t7+t30*t4+t[0];
    t39 = -t10*t7+t16*t30-t20*t4+t25*t1+t[2];
    t40 = 1/t39;
    jacmKRT[0][0] = t33*t40;
    t41 = a[3];
    t47 = -t4*t10+t16*t25-t30*t1+t20*t7+t[1];
    jacmKRT[1][0] = t41*t47*t40;
    jacmKRT[0][1] = 1.0;
    jacmKRT[1][1] = 0.0;
    jacmKRT[0][2] = 0.0;
    jacmKRT[1][2] = 1.0;
    jacmKRT[0][3] = 0.0;
    t49 = a[0];
    jacmKRT[1][3] = t49*t47*t40;
    jacmKRT[0][4] = t47*t40;
    jacmKRT[1][4] = 0.0;
    t51 = 1/t16;
    t52 = t51*t20;
    t54 = t51*t5;
    t56 = -t54*t1-t8;
    t58 = t51*t8;
    t60 = -t58*t1+t5;
    t64 = a[4];
    t66 = t51*t25;
    t70 = t51*t2;
    t71 = t1*t7;
    t73 = 2.0*t29-t66*t1+t16*t56-t60*t1-t27-t28-t70*t71;
    t75 = a[1];
    t77 = t51*t30;
    t80 = t4*t1;
    t83 = 2.0*t23-t77*t1+t16*t60+t70*t80+t56*t1+t22-t24;
    t91 = t39*t39;
    t92 = 1/t91;
    t93 = (t49*t33+t64*t47+t75*t39)*t92;
    jacmKRT[0][5] = (t49*(t3+t6+t9-t52*t1-t56*t7+t60*t4)+t64*t73+t75*t83)*t40-
t93*t83;
    t95 = t49*t41;
    t97 = a[2];
    t104 = (t95*t47+t97*t39)*t92;
    jacmKRT[1][5] = (t95*t73+t97*t83)*t40-t104*t83;
    t109 = -t70*t4+t8;
    t111 = t4*t7;
    t114 = -t58*t4-t2;
    t121 = t6+t3+t9-t66*t4-t114*t1+t109*t7;
    t128 = 2.0*t19-t77*t4+t114*t16-t109*t4-t17-t18-t54*t80;
    jacmKRT[0][6] = (t49*(2.0*t28-t52*t4+t16*t109+t54*t111+t114*t4+t27-t29)+t64
*t121+t75*t128)*t40-t93*t128;
    jacmKRT[1][6] = (t95*t121+t97*t128)*t40-t104*t128;
    t141 = -t70*t7-t5;
    t144 = -t54*t7+t2;
    t154 = 2.0*t18-t66*t7+t144*t16+t58*t71+t141*t7+t17-t19;
    t159 = t9+t3+t6-t77*t7-t141*t4+t144*t1;
    jacmKRT[0][7] = (t49*(2.0*t24-t52*t7+t16*t141-t144*t7-t22-t23-t58*t111)+t64
*t154+t75*t159)*t40-t93*t159;
    jacmKRT[1][7] = (t95*t154+t97*t159)*t40-t104*t159;
    jacmKRT[0][8] = t49*t40;
    jacmKRT[1][8] = 0.0;
    jacmKRT[0][9] = t64*t40;
    jacmKRT[1][9] = t95*t40;
    jacmKRT[0][10] = t75*t40-t93;
    jacmKRT[1][10] = t97*t40-t104;
    return;
  }
}

void BundleWrapper::calcImgProjJacS(double a[5],double v[3],double t[3],double M[3],double jacmS[2][3])
{
  double t1;
  double t10;
  double t109;
  double t11;
  double t111;
  double t12;
  double t13;
  double t15;
  double t16;
  double t18;
  double t2;
  double t20;
  double t21;
  double t22;
  double t24;
  double t27;
  double t29;
  double t3;
  double t31;
  double t33;
  double t38;
  double t4;
  double t43;
  double t48;
  double t5;
  double t51;
  double t52;
  double t6;
  double t66;
  double t7;
  double t70;
  double t71;
  double t72;
  double t75;
  double t77;
  double t84;
  double t89;
  double t90;
  double t92;
  double t93;
  double t95;
  {
    t1 = a[0];
    t2 = v[1];
    t3 = t2*t2;
    t4 = 2.0*t3;
    t5 = v[2];
    t6 = t5*t5;
    t7 = 2.0*t6;
    t10 = a[4];
    t11 = v[0];
    t12 = t11*t2;
    t13 = t11*t11;
    t15 = sqrt(1.0-t13-t3-t6);
    t16 = t15*t5;
    t18 = 2.0*t12+2.0*t16;
    t20 = a[1];
    t21 = t11*t5;
    t22 = t15*t2;
    t24 = 2.0*t21-2.0*t22;
    t27 = M[0];
    t29 = M[1];
    t31 = M[2];
    t33 = -t27*t11-t2*t29-t5*t31;
    t38 = t15*t31+t11*t29-t2*t27;
    t43 = t15*t27+t2*t31-t5*t29;
    t48 = t15*t29+t27*t5-t11*t31;
    t51 = -t33*t5+t15*t38-t43*t2+t48*t11+t[2];
    t52 = 1/t51;
    t66 = -t33*t2+t15*t48-t38*t11+t43*t5+t[1];
    t70 = t51*t51;
    t71 = 1/t70;
    t72 = (t1*(-t33*t11+t15*t43-t48*t5+t38*t2+t[0])+t10*t66+t51*t20)*t71;
    jacmS[0][0] = (t1*(1.0-t4-t7)+t10*t18+t20*t24)*t52-t72*t24;
    t75 = t1*a[3];
    t77 = a[2];
    t84 = (t75*t66+t77*t51)*t71;
    jacmS[1][0] = (t75*t18+t77*t24)*t52-t84*t24;
    t89 = 2.0*t13;
    t90 = 1.0-t89-t7;
    t92 = t2*t5;
    t93 = t15*t11;
    t95 = 2.0*t92+2.0*t93;
    jacmS[0][1] = (t1*(2.0*t12-2.0*t16)+t10*t90+t20*t95)*t52-t95*t72;
    jacmS[1][1] = (t75*t90+t95*t77)*t52-t84*t95;
    t109 = 2.0*t92-2.0*t93;
    t111 = 1.0-t89-t4;
    jacmS[0][2] = (t1*(2.0*t21+2.0*t22)+t10*t109+t20*t111)*t52-t111*t72;
    jacmS[1][2] = (t75*t109+t111*t77)*t52-t84*t111;
    return;
  }
}

void BundleWrapper::calcImgProjJacRTS(double a[5],double v[3],double t[3],double M[3],double jacmRT[2][6],double jacmS[2][3])
{
  double t1;
  double t10;
  double t102;
  double t107;
  double t109;
  double t11;
  double t112;
  double t119;
  double t12;
  double t126;
  double t13;
  double t139;
  double t142;
  double t15;
  double t152;
  double t157;
  double t16;
  double t169;
  double t17;
  double t170;
  double t173;
  double t175;
  double t177;
  double t179;
  double t18;
  double t19;
  double t192;
  double t193;
  double t195;
  double t197;
  double t2;
  double t20;
  double t21;
  double t211;
  double t213;
  double t23;
  double t25;
  double t27;
  double t29;
  double t3;
  double t33;
  double t34;
  double t36;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t5;
  double t51;
  double t53;
  double t54;
  double t57;
  double t6;
  double t60;
  double t63;
  double t69;
  double t7;
  double t70;
  double t8;
  double t84;
  double t88;
  double t89;
  double t9;
  double t90;
  double t93;
  double t95;
  {
    t1 = a[0];
    t2 = v[0];
    t3 = M[0];
    t4 = t2*t3;
    t5 = v[1];
    t6 = M[1];
    t7 = t5*t6;
    t8 = v[2];
    t9 = M[2];
    t10 = t8*t9;
    t11 = t2*t2;
    t12 = t5*t5;
    t13 = t8*t8;
    t15 = sqrt(1.0-t11-t12-t13);
    t16 = 1/t15;
    t17 = t15*t3;
    t18 = t5*t9;
    t19 = t8*t6;
    t20 = t17+t18-t19;
    t21 = t16*t20;
    t23 = t16*t6;
    t25 = -t23*t2-t9;
    t27 = t16*t9;
    t29 = -t2*t27+t6;
    t33 = a[4];
    t34 = t5*t3;
    t36 = t15*t6;
    t37 = t8*t3;
    t38 = t2*t9;
    t39 = t36+t37-t38;
    t40 = t16*t39;
    t44 = t15*t9;
    t45 = t2*t6;
    t46 = t16*t3;
    t47 = t2*t8;
    t49 = 2.0*t34-t40*t2+t15*t25-t2*t29-t44-t45-t47*t46;
    t51 = a[1];
    t53 = t44+t45-t34;
    t54 = t16*t53;
    t57 = t2*t5;
    t60 = 2.0*t37-t54*t2+t15*t29+t46*t57+t25*t2+t36-t38;
    t63 = -t4-t7-t10;
    t69 = -t63*t8+t15*t53-t20*t5+t39*t2+t[2];
    t70 = 1/t69;
    t84 = -t63*t5+t39*t15-t53*t2+t20*t8+t[1];
    t88 = t69*t69;
    t89 = 1/t88;
    t90 = (t1*(-t63*t2+t15*t20-t39*t8+t53*t5+t[0])+t33*t84+t51*t69)*t89;
    jacmRT[0][0] = (t1*(t4+t7+t10-t21*t2-t25*t8+t5*t29)+t33*t49+t51*t60)*t70-
t60*t90;
    t93 = t1*a[3];
    t95 = a[2];
    t102 = (t93*t84+t95*t69)*t89;
    jacmRT[1][0] = (t93*t49+t95*t60)*t70-t60*t102;
    t107 = -t46*t5+t9;
    t109 = t5*t8;
    t112 = -t27*t5-t3;
    t119 = t7+t4+t10-t40*t5-t112*t2+t107*t8;
    t126 = 2.0*t19-t54*t5+t15*t112-t107*t5-t17-t18-t23*t57;
    jacmRT[0][1] = (t1*(2.0*t45-t21*t5+t15*t107+t23*t109+t112*t5+t44-t34)+t119*
t33+t126*t51)*t70-t90*t126;
    jacmRT[1][1] = (t93*t119+t95*t126)*t70-t126*t102;
    t139 = -t46*t8-t6;
    t142 = -t23*t8+t3;
    t152 = 2.0*t18-t40*t8+t15*t142+t27*t47+t139*t8+t17-t19;
    t157 = t10+t4+t7-t54*t8-t139*t5+t142*t2;
    jacmRT[0][2] = (t1*(2.0*t38-t21*t8+t15*t139-t142*t8-t36-t37-t109*t27)+t33*
t152+t51*t157)*t70-t90*t157;
    jacmRT[1][2] = (t93*t152+t95*t157)*t70-t102*t157;
    jacmRT[0][3] = t1*t70;
    jacmRT[1][3] = 0.0;
    jacmRT[0][4] = t33*t70;
    jacmRT[1][4] = t93*t70;
    jacmRT[0][5] = t51*t70-t90;
    jacmRT[1][5] = t95*t70-t102;
    t169 = 2.0*t12;
    t170 = 2.0*t13;
    t173 = t15*t8;
    t175 = 2.0*t57+2.0*t173;
    t177 = t15*t5;
    t179 = 2.0*t47-2.0*t177;
    jacmS[0][0] = (t1*(1.0-t169-t170)+t33*t175+t179*t51)*t70-t90*t179;
    jacmS[1][0] = (t93*t175+t95*t179)*t70-t102*t179;
    t192 = 2.0*t11;
    t193 = 1.0-t192-t170;
    t195 = t15*t2;
    t197 = 2.0*t109+2.0*t195;
    jacmS[0][1] = (t1*(2.0*t57-2.0*t173)+t33*t193+t51*t197)*t70-t90*t197;
    jacmS[1][1] = (t93*t193+t95*t197)*t70-t102*t197;
    t211 = 2.0*t109-2.0*t195;
    t213 = 1.0-t192-t169;
    jacmS[0][2] = (t1*(2.0*t47+2.0*t177)+t211*t33+t51*t213)*t70-t90*t213;
    jacmS[1][2] = (t93*t211+t95*t213)*t70-t102*t213;
    return;
  }
}


void BundleWrapper::calcImgProjJacRT(double a[5],double v[3],double t[3],double M[3], double jacmRT[2][6])
{
  double t1;
  double t10;
  double t102;
  double t107;
  double t109;
  double t11;
  double t112;
  double t119;
  double t12;
  double t126;
  double t13;
  double t139;
  double t142;
  double t15;
  double t152;
  double t157;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t23;
  double t25;
  double t27;
  double t29;
  double t3;
  double t33;
  double t34;
  double t36;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t5;
  double t51;
  double t53;
  double t54;
  double t57;
  double t6;
  double t60;
  double t63;
  double t69;
  double t7;
  double t70;
  double t8;
  double t84;
  double t88;
  double t89;
  double t9;
  double t90;
  double t93;
  double t95;
  {
    t1 = a[0];
    t2 = v[0];
    t3 = M[0];
    t4 = t2*t3;
    t5 = v[1];
    t6 = M[1];
    t7 = t5*t6;
    t8 = v[2];
    t9 = M[2];
    t10 = t8*t9;
    t11 = t2*t2;
    t12 = t5*t5;
    t13 = t8*t8;
    t15 = sqrt(1.0-t11-t12-t13);
    t16 = 1/t15;
    t17 = t15*t3;
    t18 = t5*t9;
    t19 = t8*t6;
    t20 = t17+t18-t19;
    t21 = t16*t20;
    t23 = t16*t6;
    t25 = -t23*t2-t9;
    t27 = t16*t9;
    t29 = -t2*t27+t6;
    t33 = a[4];
    t34 = t5*t3;
    t36 = t15*t6;
    t37 = t8*t3;
    t38 = t2*t9;
    t39 = t36+t37-t38;
    t40 = t16*t39;
    t44 = t15*t9;
    t45 = t2*t6;
    t46 = t16*t3;
    t47 = t2*t8;
    t49 = 2.0*t34-t40*t2+t15*t25-t2*t29-t44-t45-t47*t46;
    t51 = a[1];
    t53 = t44+t45-t34;
    t54 = t16*t53;
    t57 = t2*t5;
    t60 = 2.0*t37-t54*t2+t15*t29+t46*t57+t25*t2+t36-t38;
    t63 = -t4-t7-t10;
    t69 = -t63*t8+t15*t53-t20*t5+t39*t2+t[2];
    t70 = 1/t69;
    t84 = -t63*t5+t39*t15-t53*t2+t20*t8+t[1];
    t88 = t69*t69;
    t89 = 1/t88;
    t90 = (t1*(-t63*t2+t15*t20-t39*t8+t53*t5+t[0])+t33*t84+t51*t69)*t89;
    jacmRT[0][0] = (t1*(t4+t7+t10-t21*t2-t25*t8+t5*t29)+t33*t49+t51*t60)*t70-
t60*t90;
    t93 = t1*a[3];
    t95 = a[2];
    t102 = (t93*t84+t95*t69)*t89;
    jacmRT[1][0] = (t93*t49+t95*t60)*t70-t60*t102;
    t107 = -t46*t5+t9;
    t109 = t5*t8;
    t112 = -t27*t5-t3;
    t119 = t7+t4+t10-t40*t5-t112*t2+t107*t8;
    t126 = 2.0*t19-t54*t5+t15*t112-t107*t5-t17-t18-t23*t57;
    jacmRT[0][1] = (t1*(2.0*t45-t21*t5+t15*t107+t23*t109+t112*t5+t44-t34)+t119*
t33+t126*t51)*t70-t90*t126;
    jacmRT[1][1] = (t93*t119+t95*t126)*t70-t126*t102;
    t139 = -t46*t8-t6;
    t142 = -t23*t8+t3;
    t152 = 2.0*t18-t40*t8+t15*t142+t27*t47+t139*t8+t17-t19;
    t157 = t10+t4+t7-t54*t8-t139*t5+t142*t2;
    jacmRT[0][2] = (t1*(2.0*t38-t21*t8+t15*t139-t142*t8-t36-t37-t109*t27)+t33*
t152+t51*t157)*t70-t90*t157;
    jacmRT[1][2] = (t93*t152+t95*t157)*t70-t102*t157;
    jacmRT[0][3] = t1*t70;
    jacmRT[1][3] = 0.0;
    jacmRT[0][4] = t33*t70;
    jacmRT[1][4] = t93*t70;
    jacmRT[0][5] = t51*t70-t90;
    jacmRT[1][5] = t95*t70-t102;
    return;
  }
}

