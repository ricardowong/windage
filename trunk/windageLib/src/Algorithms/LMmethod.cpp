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

#include "Algorithms/LMmethod.h"
using namespace windage;
using namespace windage::Algorithms;

bool LMmethod::Calculate()
{
	if(this->homography == NULL)
		return false;
	if(this->referencePoints == NULL || this->scenePoints == NULL)
		return false;

	int n = (int)this->referencePoints->size();
	if(n < 4)
		return false;

	CvLevMarq solver(8, 0, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, this->maxIteration, DBL_EPSILON));
	CvMat modelPart = cvMat(solver.param->rows, solver.param->cols, CV_64F, this->homography->m1 );
	cvCopy(&modelPart, solver.param);

	while(true)
	{
		const CvMat* _param = 0;
		CvMat* _JtJ = 0;
		CvMat* _JtErr = 0;
		double* _errNorm = 0;

		if( !solver.updateAlt( _param, _JtJ, _JtErr, _errNorm ))
			break;
		
		for(int i=0; i<n; i++)
		{
			const double* h = _param->data.db;
			double Mx = (*this->referencePoints)[i].GetPoint().x;
			double My = (*this->referencePoints)[i].GetPoint().y;
			double mx = (*this->scenePoints)[i].GetPoint().x;
			double my = (*this->scenePoints)[i].GetPoint().y;

			double ww = 1./(h[6]*Mx + h[7]*My + 1.);
			double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
			double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;

			double err[] = { _xi - mx, _yi - my };
			if( _JtJ || _JtErr )
			{
				double J[][8] =
				{
					{ Mx*ww, My*ww, ww, 0, 0, 0, -Mx*ww*_xi, -My*ww*_xi },
					{ 0, 0, 0, Mx*ww, My*ww, ww, -Mx*ww*_yi, -My*ww*_yi }
				};
				for(int j=0; j<8; j++)
				{
					for(int k=j; k<8; k++)
					{
						_JtJ->data.db[j*8+k] += J[0][j]*J[0][k] + J[1][j]*J[1][k];
					}
					_JtErr->data.db[j] += J[0][j]*err[0] + J[1][j]*err[1];
				}
			}
			if( _errNorm )
				*_errNorm += err[0]*err[0] + err[1]*err[1];
		}
	}

	cvCopy( solver.param, &modelPart );
	return true;
}
