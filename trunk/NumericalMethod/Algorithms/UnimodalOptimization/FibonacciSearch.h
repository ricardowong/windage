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

#ifndef _FIBONACCI_SEARCH_H_
#define _FIBONACCI_SEARCH_H_

#include "UnimodalOptimization.h"
#include <vector>

namespace windage
{
	class FibonacciSearch : public UnimodalOptimization
	{
	private:
		std::vector<double> fibonacciNumbers;
		int fibonacciCount;
		double xMin;
		double xMax;

	public:
		inline FibonacciSearch(){this->xMin = -1.0;this->xMax = -1.0;};
		inline ~FibonacciSearch(){};
		
		inline void SetFibonacciCount(int fibonacciCount){this->fibonacciCount = fibonacciCount;};
		inline void SetInitialValue(long double xMin, long double xMax){this->xMin = xMin; this->xMax = xMax;};
		bool Calculate(long double* solution1, long double* solution2);
	};
}

#endif