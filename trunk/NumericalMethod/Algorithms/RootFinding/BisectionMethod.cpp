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


#include "BisectionMethod.h"

using namespace windage;
bool BisectionMethod::Calculate(long double* solution)
{
	this->repeat = 1;

	// fault initialization
	if(!function)
	{
		(*solution) = -2;
		return false;
	}

	long double localXMin = this->xMin;
	long double localXMax = this->xMax;

	
	long double minResult = function(localXMin);
	long double maxResult = function(localXMax);

	// fault initial value
	if(minResult * maxResult > 0)
	{
		(*solution) = -3;
		return false;
	}

	// calculate
	bool processing = true;
	while(processing)
	{
		if(abs(minResult) < LEAST_ERROR_RANGE)
		{
			(*solution) = localXMin;
			return true;
		}
		if(abs(maxResult) < LEAST_ERROR_RANGE)
		{
			(*solution) = localXMax;
			return true;
		}

		minResult = function(localXMin);
		maxResult = function(localXMax);
		
		long double mid = (localXMin + localXMax) * 0.5;
		long double midResult = function(mid);

		if(midResult * minResult > 0)
		{
			localXMin = mid;
			minResult = midResult;
		}
		else
		{
			localXMax = mid;
			maxResult = midResult;
		}

		if(localXMax-localXMin < LEAST_ERROR_RANGE)
		{
			(*solution) = localXMin;
			return true;
		}

		this->repeat++;
		if(this->repeat > MAX_INTERATE_TIME)
		{
			(*solution) = abs(minResult)<abs(maxResult)?localXMin:localXMax;
			return false;
		}
	}

	(*solution) = 0;
	return false;
}