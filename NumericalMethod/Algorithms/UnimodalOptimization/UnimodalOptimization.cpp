/* ========================================================================
 * PROJECT: Numerical Optimization for windage Library
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

#include "UnimodalOptimization.h"

#include <vector>
#include <time.h>

using namespace windage;
int UnimodalOptimization::SeekBound(double* xMin, double* xMax)
{
	std::vector<double> params;			params.resize(1);
	srand(time(NULL));

	double x = (double)rand() - RAND_MAX/2;
	double d = (double)rand() + DELTA;

	double xMinus = x - d;
	double xPlus = x + d;	
	
	params[0] = xMinus;
	double solutionMinus = function(&params, 1);
	params[0] = x;
	double solution		 = function(&params, 1);
	params[0] = xPlus;
	double solutionPlus  = function(&params, 1);

	std::vector<double> xList;
	std::vector<double> solutionList;
		
	int iteration = 0;

	bool processing = true;
	while(processing)
	{
		xList.clear();
		solutionList.clear();

		if(solutionMinus >= solution && solution >= solutionPlus)
		{
			
		}
		if(solutionMinus <= solution && solution <= solutionPlus)
		{
			d = -d;
		}
		if(solutionMinus >= solution && solution <= solutionPlus)
		{
			(*xMin) = xMinus;
			(*xMax) = xPlus;
			return iteration;
		}

		// k = -1
		xList.push_back(x - d);			
		params[0] = x - d;
		solutionList.push_back(function(&params, 1));
		// k = 0
		xList.push_back(x);
		params[0] = x;
		solutionList.push_back(function(&params, 1));
		// k = +1
		xList.push_back(x + d);
		params[0] = x + d;
		solutionList.push_back(function(&params, 1));

		for(int k=1; k<1000; k++)
		{
			double tempX = xList[k+1] + pow(2.0, k) * d;

			params[0] = tempX;
			double tempSolution = function(&params, 1);

			xList.push_back(tempX);
			solutionList.push_back(tempSolution);

			if(solutionList[k+2] >= solutionList[k+1] && d > 0)
			{
				(*xMin) = xList[k];
				(*xMax) = xList[k+2];
				return iteration;
			}
			if(solutionList[k+2] >= solutionList[k+1] && d < 0)
			{
				(*xMin) = xList[k+2];
				(*xMax) = xList[k];
				return iteration;
			}
		}

		xMinus = x - d;
		xPlus = x + d;

		params[0] = xMinus;
		solutionMinus = function(&params, 1);
		params[0] = x;
		solution	  = function(&params, 1);
		params[0] = xPlus;
		solutionPlus  = function(&params, 1);

		iteration++;
		if(iteration > MAX_INTERATE_TIME)
		{
			(*xMin) = xMinus;
			(*xMax) = xPlus;
			return false;
		}
	}

	return -1;
}
