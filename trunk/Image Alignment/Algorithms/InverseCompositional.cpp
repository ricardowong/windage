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

#include "InverseCompositional.h"
using namespace windage;

bool InverseCompositional::AttatchTemplateImage(IplImage* image)
{
	if(this->templateImage == NULL)
		return false;
	if( this->templateImage->width != image->width ||
		this->templateImage->height != image->height)
		return false;

	cvCopyImage(image, this->templateImage);

	return true;
}

bool InverseCompositional::Initialize()
{
	// Calculate gradient of T.
	cvSobel(this->templateImage, pGradTx, 1, 0); // Gradient in X direction
	cvSobel(this->templateImage, pGradTy, 0, 1); // Gradient in Y direction

	// Compute steepest descent images and Hessian
	cvSet(H, cvScalar(0)); // Set Hessian with zeroes

	// Walk through pixels in the template T.
	for(int y=0; y<this->height; y++)
	{
		for(int x=0; x<this->width; x++)
		{
			// Evaluate gradient of T.
			double Tx = cvGetReal2D(pGradTx, y, x);
			double Ty = cvGetReal2D(pGradTy, y, x);

			// Calculate steepest descent image's element.
			double stdesc[4];
			stdesc[0] = -Tx*y+Ty*x;
			stdesc[1] = Tx;
			stdesc[2] = Ty;
			stdesc[3] = Tx*x+Ty*y;

			cvSet2D(pStDesc, y, x, cvScalar(stdesc[0], stdesc[1], stdesc[2], stdesc[3]));

			// Add a term to Hessian.
			for(int l=0; l<4; l++)
			{
				for(int m=0; m<4; m++)
				{
					double value = cvGetReal2D(H, l, m);
					cvSetReal2D(H, l, m, value + (stdesc[l] * stdesc[m]));
				} 
			}
		}	
	}

	// Invert Hessian.
	double inv_res = cvInvert(H, iH);
	if(inv_res==0)
	{
		printf("Error: Hessian is singular.\n");
		return false;
	}

	return true;
}

double InverseCompositional::UpdateHomography(IplImage* image, double* delta)
{
	double error = 0.0;
	cvSetIdentity(W);

	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			cvSetReal2D(W, y, x , this->homography.m[y][x]);
		}
	}

	int ix, iy;
	int pixel_count=0; // Count of processed pixels
	cvSet(b, cvScalar(0)); // Set b matrix with zeroes

	// Walk through pixels in the template T.
	for(int y=0; y<this->height; y++)
	{
		for(int x=0; x<this->width; x++)
		{
			// get value
			windage::Vector3 point(x, y, 1.0);
			windage::Vector3 out = this->homography * point;
			out /= out.z;

			ix = cvRound(out.x);
			iy = cvRound(out.y);

			double value = -1.0;
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				value = cvGetReal2D(image, iy, ix);
			cvSetReal2D(samplingImage, y, x, value);

			if(0 <= ix && ix < image->width &&
				0 <= iy && iy < image->height)
			{
				pixel_count++;

				// Calculate image difference D = I(W(x,p))-T(x).
				double D = cvGetReal2D(image, iy, ix) - cvGetReal2D(this->templateImage, y, x);

				// Update mean error value.
				error += abs(D);

				// Add a term to b matrix.
				CvScalar stdesc = cvGet2D(pStDesc, y, x);
				for(int i=0; i<4; i++)
				{
					double value = cvGetReal1D(b, i);
					cvSetReal1D(b, i, value + (stdesc.val[i] * D));
				} 
			}
		}
	}

	// Finally, calculate mean_error.
	if(pixel_count!=0)
		error /= pixel_count;

	// Find parameter increment. 
	cvGEMM(iH, b, 1, 0, 0, delta_p);
	float delta_wz = CV_MAT_ELEM(*delta_p, float, 0, 0) * (float)PARAMETER_AMPLIFICATION;
	float delta_tx = CV_MAT_ELEM(*delta_p, float, 1, 0) * (float)PARAMETER_AMPLIFICATION;
	float delta_ty = CV_MAT_ELEM(*delta_p, float, 2, 0) * (float)PARAMETER_AMPLIFICATION;
	float delta_s  = CV_MAT_ELEM(*delta_p, float, 3, 0) * (float)PARAMETER_AMPLIFICATION;

	init_warp(dW, delta_wz, delta_tx, delta_ty, 1.0f+delta_s);
	// Invert warp.
	double inv_res = cvInvert(dW, idW);
	if(inv_res==0)
	{
		printf("Error: Warp matrix is singular.\n");
		return -1.0;
	} 

	cvGEMM(W, idW, 1, 0, 0, dW);
	cvCopy(dW, W);

	// update homography
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			this->homography.m[y][x] = cvGetReal2D(W, y, x);
		}
	}

	// return delta factor
	if(delta)
	{
		(*delta) = abs(delta_wz) + abs(delta_tx) + abs(delta_ty) + abs(delta_s);
	}

	return error;
}

void InverseCompositional::init_warp(CvMat* W, float wz, float tx, float ty, float s)
{
	CV_MAT_ELEM(*W, float, 0, 0) = s;
	CV_MAT_ELEM(*W, float, 1, 0) = wz;
	CV_MAT_ELEM(*W, float, 2, 0) = 0;

	CV_MAT_ELEM(*W, float, 0, 1) = -wz;
	CV_MAT_ELEM(*W, float, 1, 1) = s;
	CV_MAT_ELEM(*W, float, 2, 1) = 0;

	CV_MAT_ELEM(*W, float, 0, 2) = tx;
	CV_MAT_ELEM(*W, float, 1, 2) = ty;
	CV_MAT_ELEM(*W, float, 2, 2) = 1;
}