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

#include "FibonacciSearch.h"

using namespace windage;
bool FibonacciSearch::Calculate(long double* solution1, long double* solution2)
{
	this->repeat = 1;

	this->fibonacciNumbers.clear();

	// generate fibonacci numbers
	this->fibonacciNumbers.push_back(1.0);
	this->fibonacciNumbers.push_back(1.0);
	for(int i=2; i<this->fibonacciCount+1; i++)
	{
		this->fibonacciNumbers.push_back(this->fibonacciNumbers[i-2] + this->fibonacciNumbers[i-1]);
	}

	double localXMin = this->xMin;
	double localXMax = this->xMax;

	int n = this->fibonacciCount;

	double length = localXMax - localXMin;
	double x1 = localXMin + fibonacciNumbers[n-2]/fibonacciNumbers[n] * length;
	double x2 = localXMax - fibonacciNumbers[n-2]/fibonacciNumbers[n] * length;

	double solutionX1 = function(x1);
	double solutionX2 = function(x2);

	while(n > 1)
	{
		// calculate
		length = localXMax - localXMin;
		double tempX1 = localXMin + (fibonacciNumbers[n-2]/fibonacciNumbers[n]) * length;
		double tempX2 = localXMax - (fibonacciNumbers[n-2]/fibonacciNumbers[n]) * length;

		// always x1 != x2
		if(tempX1 == tempX2)
			tempX2 += windage::DELTA;
//*
		if(tempX1 == x1)
		{
			x1 = tempX1;
			x2 = tempX2;

//			solutionX1 = function(x1);
			solutionX2 = function(x2);
		}
		else if(tempX2 == x2)
		{
			x1 = tempX1;
			x2 = tempX2;

			solutionX1 = function(x1);
//			solutionX2 = function(x2);
		}
		else if(tempX1 == x2)
		{
			x1 = tempX1;
			x2 = tempX2;

			solutionX1 = solutionX2;
			solutionX2 = function(x2);
		}
		else if(tempX2 == x1)
		{
			x1 = tempX1;
			x2 = tempX2;

			solutionX2 = solutionX1;
			solutionX1 = function(x1);
		}
		else
//*/
		{
			x1 = tempX1;
			x2 = tempX2;

			solutionX1 = function(x1);
			solutionX2 = function(x2);
		}

		if(solutionX1 > solutionX2)
		{
			localXMin = x1;
		}
		else if(solutionX1 < solutionX2)
		{
			localXMax = x2;
		}
		n--;

		this->repeat++;
	}

	(*solution1) = localXMin;
	(*solution2) = localXMax;
	return true;
}