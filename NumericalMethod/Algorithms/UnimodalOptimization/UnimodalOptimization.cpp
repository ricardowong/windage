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
	srand(time(NULL));

	double x = (double)rand();
	double d = (double)rand();

	double xMinus = x - d;
	double xPlus = x + d;	
	
	double solutionMinus = function(xMinus);
	double solution		 = function(x);
	double solutionPlus  = function(xPlus);

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
			xList.push_back(x - d);			
			solutionList.push_back(function(x - d));
		}
		if(solutionMinus <= solution && solution <= solutionPlus)
		{
			d = -d;
			xList.push_back(x - d);			
			solutionList.push_back(function(x - d));
		}
		if(solutionMinus >= solution && solution <= solutionPlus)
		{
			(*xMin) = xMinus;
			(*xMax) = xPlus;
			return iteration;
		}

		xList.push_back(x);
		solutionList.push_back(function(x));

		for(int k=1; k<2; k++)
		{
			double tempX = xList[k-1] + pow(2.0, k-1) * d;
			double tempSolution = function(tempX);

			xList.push_back(tempX);
			solutionList.push_back(tempSolution);

			if(solutionList[k] >= solutionList[k-1] && d > 0)
			{
				(*xMin) = xList[k-1];
				(*xMax) = xList[k+1];
				return iteration;
			}
			if(solutionList[k] >= solutionList[k-1] && d < 0)
			{
				(*xMin) = xList[k+1];
				(*xMax) = xList[k-1];
				return iteration;
			}
		}

		xMinus = x - d;
		xPlus = x + d;

		solutionMinus = function(xMinus);
		solution	  = function(x);
		solutionPlus  = function(xPlus);

		iteration++;
	}

	return -1;
}
