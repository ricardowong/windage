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
			double I1 = -1.0;
			double I2 = -1.0;

			// se
			double value = cvGetReal2D(templateImage, y, x);
			se[index] = value;

			// d(I(p))/d(p) (1x2 gradient of image)
			Vector2 tempdI;
			I1 = cvGetReal2D(templateImage, y, x - DELTA);
			I2 = cvGetReal2D(templateImage, y, x + DELTA);
			tempdI.x = (I2 - I1)/(2*DELTA);

			I1 = cvGetReal2D(templateImage, y - DELTA, x);
			I2 = cvGetReal2D(templateImage, y + DELTA, x);
			tempdI.y = (I2 - I1)/(2*DELTA);

			dI[index] = tempdI;

			// dw(x) / dx (2xp jacobian matrix)
			point1.x = x;
			point1.y = y;

			// TODO! replace homography derivation
			for(int i=0; i<this->p; i++)
			{
				windage::Matrix3 tempHomography1 = this->homography;
				windage::Matrix3 tempHomography2 = this->homography;

				tempHomography1.m1[i] -= HOMOGRAPHY_DELTA;
				tempHomography2.m1[i] += HOMOGRAPHY_DELTA;

				out1 = tempHomography1 * point1;
				out2 = tempHomography2 * point1;
				out1 = (out2 - out1)/(2*DELTA);

				windage::Vector2 temp(out1.x, out1.y);
				dwx[index][i] = temp;
			}

			index++;
		}
	}

	return true;
}

double HomographyESM::UpdateHomography(IplImage* image, double* delta)
{
	if(this->templateImage == NULL)
		return false;
	if(image == NULL)
		return false;
	if(image->nChannels != 1)
		return false;

	// update homography
	Vector3 point, point1, point2, out, out1, out2;
	point.z = 1.0;
	point1.z = 1.0;
	point2.z = 1.0;

	int ix, iy;
	int index = 0;
	for(int y=DELTA; y<templateImage->height-DELTA; y+= SAMPLING_STEP)
	{
		for(int x=DELTA; x<templateImage->width-DELTA; x+= SAMPLING_STEP)
		{
			point.x = x;
			point.y = y;
			out = this->homography * point;
			out /= out.z;

			ix = cvRound(out.x);
			iy = cvRound(out.y);

			// sxc
			double value = -1.0;
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				value = cvGetReal2D(image, iy, ix);
			sxc[index] = value;

			// for debuging
			cvSetReal2D(samplingImage, y, x, value);

			double I1 = -1.0;
			double I2 = -1.0;

			// TODO! : replace to sobel image
			// dI(w(p)) / dp (1x2 gradient of image)
			Vector2 tempdwI;

			point1.x = x - DELTA;
			point1.y = y;
			point2.x = x + DELTA;
			point2.y = y;
			out1 = this->homography * point1;
			out2 = this->homography * point2;
			out1 /= out1.z;
			out2 /= out2.z;

			I1 = -1.0;
			I2 = -1.0;
			ix = cvRound(out1.x);
			iy = cvRound(out1.y);
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				I1 = cvGetReal2D(image, iy, ix);
			ix = cvRound(out2.x);
			iy = cvRound(out2.y);
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				I2 = cvGetReal2D(image, iy, ix);
			tempdwI.x = (I2 - I1)/(2*DELTA);

			point1.x = x;
			point1.y = y - DELTA;
			point2.x = x;
			point2.y = y + DELTA;
			out1 = this->homography * point1;
			out2 = this->homography * point2;
			out1 /= out1.z;
			out2 /= out2.z;

			I1 = -1.0;
			I2 = -1.0;
			ix = cvRound(out1.x);
			iy = cvRound(out1.y);
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				I1 = cvGetReal2D(image, iy, ix);
			ix = cvRound(out2.x);
			iy = cvRound(out2.y);
			if( 0 < ix && ix < image->width && 0 < iy && iy < image->height)
				I2 = cvGetReal2D(image, iy, ix);
			tempdwI.y = (I2 - I1)/(2*DELTA);

			// Jsum = J(e) + J(xc)
			for(int i=0; i<this->p; i++)
			{
				double value = (dI[index] + dwI[index]) * dwx[index][i];
				cvSetReal2D(JacobianSum, index, i, value);
			}

			index++;
		}
	}

	// delta_s
	double error = 0.0;
	for(int i=0; i<this->q; i++)
	{
		double value = (sxc[i] - se[i]);
		cvSetReal2D(dS, i, 0, value);
		error += abs(value);
	}
	error /= this->q ;

	// pseudo-invers
	cvTranspose(JacobianSum, JacobianSumT);
	cvMatMul(JacobianSumT, JacobianSum, Jacobian);
	cvMatMul(JacobianSumT, dS, JacobianTdS);

	double inv_res = cvInvert(Jacobian, JacobianInvers, CV_SVD_SYM);
	cvMatMul(JacobianInvers, JacobianTdS, dx);

	// update homography
	double tempDelta = 0.0;
	for(int i=0; i<this->p; i++)
	{
		double value = -2.0 * cvGetReal1D(dx, i) * PARAMETER_AMPLIFICATION;
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