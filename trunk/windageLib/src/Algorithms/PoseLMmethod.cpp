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

#include <cv.h>

#include "Algorithms/PoseLMmethod.h"
using namespace windage;
using namespace windage::Algorithms;

bool PoseLMmethod::Calculate()
{
	if(this->calibration == NULL)
		return false;
	if(this->referencePoints == NULL || this->scenePoints == NULL)
		return false;
	if(this->referencePoints->size() != this->scenePoints->size())
		return false;

	int count = 0;
	for(unsigned int i=0; i<this->referencePoints->size(); i++)
	{
		if((*this->referencePoints)[i].IsOutlier() == false)
			count++;
	}
	if(count < 6)
		return false;

	//https://code.ros.org/trac/opencv/browser/trunk/opencv/src/cv/cvcalibration.cpp :1112
	double a[9];
	double R[9];
	double param[6];

	CvMat _A = cvMat( 3, 3, CV_64F, a );
	CvMat _R = cvMat( 3, 3, CV_64F, R );
	CvMat _r = cvMat( 3, 1, CV_64F, param );
	CvMat _t = cvMat( 3, 1, CV_64F, param + 3 );
	CvMat _param = cvMat( 6, 1, CV_64F, param );
	CvMat _dpdr, _dpdt;
	
	cvCopy(this->calibration->GetIntrinsicMatrix(), &_A);
	CvMat* distortionCoefficients = this->calibration->GetDistortionCoefficients();
	CvMat* extrinsic = this->calibration->GetExtrinsicMatrix();
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			CV_MAT_ELEM(_R, double, y, x) = CV_MAT_ELEM((*extrinsic), double, y, x);
		}
		CV_MAT_ELEM(_t, double, y, 0) = CV_MAT_ELEM((*extrinsic), double, y, 3);
	}
	cvRodrigues2(&_R, &_r);

	cv::Ptr<CvMat> _M = cvCreateMat(1, count, CV_64FC3);
	cv::Ptr<CvMat> _m = cvCreateMat(1, count, CV_64FC2);
	int index = 0;
	for(int i=0; i<count; i++)
	{
		if((*this->referencePoints)[i].IsOutlier() == false)
		{
			windage::Vector3 ref = (*this->referencePoints)[i].GetPoint();
			windage::Vector3 sce = (*this->scenePoints)[i].GetPoint();

			((CvPoint3D64f*)_M->data.db)[index] = cvPoint3D64f(ref.x, ref.y, ref.z);
			((CvPoint2D64f*)_m->data.db)[index] = cvPoint2D64f(sce.x, sce.y);
			index++;
		}
	}

	CvLevMarq solver(6, count*2, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, this->maxIteration, FLT_EPSILON), true);
	cvCopy( &_param, solver.param );

	while(true)
	{
		CvMat* _J = 0;
		CvMat* _err = 0;
		const CvMat* __param = 0;
		bool proceed = solver.update(__param, _J, _err);
		cvCopy(__param, &_param);
		if(!proceed || !_err)
			break;

		cvReshape(_err, _err, 2, 1);
		if(_J)
		{
			cvGetCols(_J, &_dpdr, 0, 3 );
			cvGetCols(_J, &_dpdt, 3, 6 );
			cvProjectPoints2( _M, &_r, &_t, &_A, distortionCoefficients, _err, &_dpdr, &_dpdt, 0, 0, 0);
		}
		else
		{
			cvProjectPoints2( _M, &_r, &_t, &_A, distortionCoefficients, _err, 0, 0, 0, 0, 0);
		}

		cvSub(_err, _m, _err);
		cvReshape( _err, _err, 1, 2*count );
	}
	cvCopy(solver.param, &_param);

	// update
	cvRodrigues2(&_r, &_R);
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			CV_MAT_ELEM((*extrinsic), double, y, x) = CV_MAT_ELEM(_R, double, y, x);
		}
		CV_MAT_ELEM((*extrinsic), double, y, 3) = CV_MAT_ELEM(_t, double, y, 0);
	}

	return true;
}
