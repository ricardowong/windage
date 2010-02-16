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

#include "homographyESM.h"
using namespace windage;

#include <omp.h>

bool HomographyESM::AttatchTemplateImage(IplImage* image)
{
	if(this->templateImage == NULL)
		return false;
	if( this->templateImage->width != image->width ||
		this->templateImage->height != image->height)
		return false;

	cvCopyImage(image, this->templateImage);

	return true;
}

bool HomographyESM::Initialize()
{
	dI.clear();
	dI.resize(q);
	dwI.clear();
	dwI.resize(q);
	dwx.clear();
	dwx.resize(q);
	for(int i=0; i<q; i++)
		dwx[i].resize(p);

	se.clear();
	se.resize(q);
	sxc.clear();
	sxc.resize(q);

	// initialze d(I(p))/d(p) & d(w(x))/d(x)
	Vector3 point1, point2, out1, out2;
	point1.z = 1.0;
	point2.z = 1.0;

	int index = 0;
	for(int y=DELTA; y<templateImage->height-DELTA; y+= SAMPLING_STEP)
	{
		for(int x=DELTA; x<templateImage->width-DELTA; x+= SAMPLING_STEP)
		{
			float I1 = -1.0;
			float I2 = -1.0;

			// se
			float value = (float)CV_IMAGE_ELEM(templateImage, unsigned char, y, x);
			se[index] = value;

			// d(I(p))/d(p) (1x2 gradient of image)
			Vector2 tempdI;
			I1 = (float)CV_IMAGE_ELEM(templateImage, unsigned char, y, x - DELTA);
			I2 = (float)CV_IMAGE_ELEM(templateImage, unsigned char, y, x + DELTA);
			tempdI.x = (I2 - I1)/(2*DELTA);

			I1 = (float)CV_IMAGE_ELEM(templateImage, unsigned char, y - DELTA, x);
			I2 = (float)CV_IMAGE_ELEM(templateImage, unsigned char, y + DELTA, x);
			tempdI.y = (I2 - I1)/(2*DELTA);

			dI[index] = tempdI;

			// dw(x) / dx (2xp jacobian matrix)
			point1.x = x;
			point1.y = y;
			point1.z = 1.0;

			// TODO! replace homography derivation
			for(int i=0; i<this->p; i++)
			{
				windage::Matrix3 tempHomography1 = this->homography;
				windage::Matrix3 tempHomography2 = this->homography;

				tempHomography1.m1[i] -= HOMOGRAPHY_DELTA;
				tempHomography2.m1[i] += HOMOGRAPHY_DELTA;

				out1 = tempHomography1 * point1;
				out2 = tempHomography2 * point1;
				out1 /= out1.z;
				out2 /= out2.z;
				out1 = (out2 - out1)/(2*HOMOGRAPHY_DELTA);

				windage::Vector2 temp(out1.x, out1.y);
				dwx[index][i] = temp;
			}

			index++;
		}
	}

	isInitialize = true;
	return true;
}

float HomographyESM::UpdateHomography(IplImage* image, float* delta)
{
	if(isInitialize == false)
		return -1.0;
	if(this->templateImage == NULL)
		return -1.0;
	if(image == NULL)
		return -1.0;
	if(image->nChannels != 1)
		return -1.0;

	// update homography
	int index = 0;
//	#pragma omp parallel for 
	for(int y=DELTA; y<templateImage->height-DELTA; y+= SAMPLING_STEP)
	{
		Vector3 point, out;
		int ix, iy;
		Vector2 tempdwI;
		unsigned char I1 = 0;
		unsigned char I2 = 0;
		unsigned char value = 0;

		for(int x=DELTA; x<templateImage->width-DELTA; x+= SAMPLING_STEP)
		{
//			int index = ((y-DELTA)/SAMPLING_STEP) * ((templateImage->width-DELTA-1)/SAMPLING_STEP) + ((x-DELTA)/SAMPLING_STEP);

			point.x = x;
			point.y = y;
			point.z = 1.0;
			out = this->homography * point;
			float ww = 1.0f/(float)out.z;
			out *= ww;

			ix = (int)(out.x);
			iy = (int)(out.y);
			ww = 1.0f/DELTA;

			// sxc
			if( 0 < ix && ix+DELTA < image->width && 0 < iy && iy+DELTA < image->height)
			{
				value = CV_IMAGE_ELEM(image, unsigned char, iy, ix);
				I1 = CV_IMAGE_ELEM(image, unsigned char, iy, ix+DELTA);
				I2 = CV_IMAGE_ELEM(image, unsigned char, iy+DELTA, ix);
			}
			sxc[index] = (float)value;
			tempdwI.x = (float)(I1 - value)*ww;
			tempdwI.y = (float)(I2 - value)*ww;

			// for debuging
			CV_IMAGE_ELEM(samplingImage, unsigned char, y, x) = value;

			// Jsum = J(e) + J(xc)
			windage::Vector2 tempJ = (dI[index] + tempdwI);
			for(int i=0; i<this->p; i++)
			{
				CV_MAT_ELEM((*JacobianSum), float, index, i) = (float)(tempJ * dwx[index][i]);
			}

			index++;
		}
	}

	// delta_s
	float error = 0.0;
	for(int i=0; i<this->q; i++)
	{
		float value = (sxc[i] - se[i]);
		CV_MAT_ELEM((*dS), float, i, 0) = value;
		error += abs(value);
	}
	error /= this->q ;

	// pseudo-invers
//*
	cvTranspose(JacobianSum, JacobianSumT);
	cvMatMul(JacobianSumT, JacobianSum, Jacobian);
	cvMatMul(JacobianSumT, dS, JacobianTdS);

	double inv_res = cvInvert(Jacobian, JacobianInvers, CV_CHOLESKY);
	cvMatMul(JacobianInvers, JacobianTdS, dx);
//*/
/*
	cvInvert(JacobianSum, JacobianSumT, cv::DECOMP_SVD);
	cvMatMul(JacobianSumT, dS, JacobianTdS);
//*/
	// update homography
	float tempDelta = 0.0;
	for(int i=0; i<this->p; i++)
	{
		float value = -2.0f * CV_MAT_ELEM((*dx), float, 0, i) * PARAMETER_AMPLIFICATION;
		this->homography.m1[i] += value;

		tempDelta += abs(value);
	}

	// return delta factor
	if(delta != NULL)
	{
		(*delta) = tempDelta;
	}

	return error;
}