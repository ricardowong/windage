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

#ifndef _INVERSE_COMPOSITIONAL_H_
#define _INVERSE_COMPOSITIONAL_H_

#include <vector>
#include <cv.h>

#include "wMatrix.h"

#define SET_VECTOR(X, u, v)\
	CV_MAT_ELEM(*(X), float, 0, 0) = (float)(u);\
	CV_MAT_ELEM(*(X), float, 1, 0) = (float)(v);\
	CV_MAT_ELEM(*(X), float, 2, 0) = 1.0f;

#define GET_VECTOR(X, u, v)\
	(u) = (int)CV_MAT_ELEM(*(X), float, 0, 0);\
	(v) = (int)CV_MAT_ELEM(*(X), float, 1, 0);
 
namespace windage
{
	class InverseCompositional
	{
	private:
		int DELTA;
		double EPS;
		static const int MAX_ITERATOR = 100;
		static const int HOMOGRAPHY_COUNT = 9;
		Matrix3 homography;

		int width;
		int height;

		IplImage* templateImage;
		IplImage* samplingImage;

		IplImage* pGradTx;	// Gradient of I in X direction.
		IplImage* pGradTy;	// Gradient of I in Y direction.
		IplImage* pStDesc;	// Steepest descent images.

		CvMat* W;	// Current value of warp W(x,p)
		CvMat* dW;	// Warp update.
		CvMat* idW;	// Inverse of warp update.
		CvMat* X;	// Point in coordinate frame of T.
		CvMat* Z;	// Point in coordinate frame of I.

		CvMat* H;	// Hessian.
		CvMat* iH;	// Inverse of Hessian.
		CvMat* b;	// Vector in the right side of the system of linear equations.
		CvMat* delta_p;	// Parameter update value.

		void init_warp(CvMat* W, float wz, float tx, float ty, float s);
	public:
		InverseCompositional(int width=150, int height=150)
		{
			this->DELTA = 1;
			this->EPS = 1E-5f;
			this->width = width;
			this->height = height;

			homography.m1[0] = 1.0; homography.m1[1] = 0.0; homography.m1[2] = 0.0;
			homography.m1[3] = 0.0; homography.m1[4] = 1.0; homography.m1[5] = 0.0;
			homography.m1[6] = 0.0; homography.m1[7] = 0.0; homography.m1[8] = 1.0;

			// templateImage is gray
			templateImage = cvCreateImage(this->GetTemplateSize(), IPL_DEPTH_8U, 1);
			samplingImage = cvCreateImage(this->GetTemplateSize(), IPL_DEPTH_8U, 1);

			// Create matrices.
			pGradTx = cvCreateImage(cvSize(width, height), IPL_DEPTH_16S, 1);
			pGradTy = cvCreateImage(cvSize(width, height), IPL_DEPTH_16S, 1);
			pStDesc = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
			
			W = cvCreateMat(3, 3, CV_32F);
			dW = cvCreateMat(3, 3, CV_32F);
			idW = cvCreateMat(3, 3, CV_32F);
			X = cvCreateMat(3, 1, CV_32F);
			Z = cvCreateMat(3, 1, CV_32F);

			H = cvCreateMat(4, 4, CV_32F);
			iH = cvCreateMat(4, 4, CV_32F);
			b = cvCreateMat(4, 1, CV_32F);
			delta_p = cvCreateMat(4, 1, CV_32F);
		}
		~InverseCompositional()
		{
			if(templateImage)	cvReleaseImage(&templateImage);
			if(samplingImage)	cvReleaseImage(&samplingImage);

			if(pGradTx)	cvReleaseImage(&pGradTx);
			if(pGradTy)	cvReleaseImage(&pGradTy);
			if(pStDesc)	cvReleaseImage(&pStDesc);

			if(W)	cvReleaseMat(&W);
			if(dW)	cvReleaseMat(&dW);
			if(idW)	cvReleaseMat(&idW);
			if(X)	cvReleaseMat(&X);
			if(Z)	cvReleaseMat(&Z);

			if(H)	cvReleaseMat(&H);
			if(iH)	cvReleaseMat(&iH);
			if(b)	cvReleaseMat(&b);
			if(delta_p) cvReleaseMat(&delta_p);
		}

		inline Matrix3 GetHomography(){return this->homography;};
		inline CvSize GetTemplateSize(){return cvSize(this->width, this->height);};
		inline void SetInitialHomography(Matrix3 homography){this->homography = homography;};
		inline IplImage* GetTemplateImage(){return this->templateImage;};
		inline IplImage* GetSamplingImage(){return this->samplingImage;};

		bool AttatchTemplateImage(IplImage* image);
		bool Initialize();
		double UpdateHomography(IplImage* image);
	};
}

#endif