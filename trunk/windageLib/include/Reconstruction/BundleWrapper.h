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

/**
 * @file	BundleWrapper.h
 * @author	Kiyoung Kim (kkim@gist.ac.kr)
 * @version 1.0
 * @brief	
 *		Perform Bundle adjustment using Sparse Bundle Adjustment Algorithm .
 *		borrow from kcvlib
*/

#ifndef _BUNDLE_WRAPPER_H_
#define _BUNDLE_WRAPPER_H_

#include "base.h"

#include <cv.h>
#include <sba.h>

namespace windage
{
	namespace Reconstruction
	{
		const double SBA_MAX_REPROJ_ERROR = 4.0; // max motion only reprojection error

		class DLLEXPORT BundleWrapper
		{
		private:
			int m_nPT;    /** # of points */
			int m_nImage; /** # of images */

			CvMat  *m_pt3D;
			CvMat **m_pt2D;
			CvMat **m_RT;

			double m_intrinsicsba[5];
			double m_opts[SBA_OPTSSZ], m_info[SBA_INFOSZ], phi;
			int m_cnp; 
			int m_pnp;
			int m_mnp; 
			int m_itmax;

		public:
			BundleWrapper(void);
			virtual ~BundleWrapper(void);

			/**
			 * Set required parameters 
			*/
			void SetParameters(CvMat *intrinsic, CvMat *pt3D, CvMat **pt2D, CvMat **RT, int nImage, int nPTs);
			
			bool Run();

			void Matrix2Quaternion(CvMat *mat, double *q);
			void Quaternion2Matrix(double *q, CvMat *mat);
			void quat2vec(double *inp, int nin, double *outp, int nout);
			void vec2quat(double *inp, int nin, double *outp, int nout);

			static void img_projsRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata);
			static void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata);

			static void calcImgProj(double a[5],double v[3],double t[3],double M[3],double n[2]);
			static void calcImgProjJacKRTS(double a[5],double v[3],double t[3],double M[3], double jacmKRT[2][11],double jacmS[2][3]);
			static void calcImgProjJacKRT(double a[5],double v[3],double t[3],double M[3], double jacmKRT[2][11]);
			static void calcImgProjJacS(double a[5],double v[3],double t[3],double M[3], double jacmS[2][3]);
			static void calcImgProjJacRTS(double a[5],double v[3],double t[3],double M[3], double jacmRT[2][6],double jacmS[2][3]);
			static void calcImgProjJacRT(double a[5],double v[3],double t[3],double M[3], double jacmRT[2][6]);
		};
	}
}
#endif // _BUNDLE_WRAPPER_H_