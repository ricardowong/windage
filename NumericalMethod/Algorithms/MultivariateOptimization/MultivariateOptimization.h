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


#ifndef _MULTIVARIATE_OPTIMIZATION_H_
#define _MULTIVARIATE_OPTIMIZATION_H_

#include <math.h>
#include "base.h"

#include "Utils/Logger.h"

namespace windage
{
	class MultivariateOptimization
	{
	protected:
		int repeat;
		function_pointer function;

		Logger* trace;

	public:
		inline MultivariateOptimization(){this->repeat = 0; this->function = 0;};
		virtual inline ~MultivariateOptimization(){};

		inline void AttatchFunction(function_pointer function){this->function = function;};
		inline void AttatchTrace(Logger* trace){this->trace = trace;};
		inline int GetRepatCount(){return this->repeat;};

		/*
		 * Can not found Case (false - return solution value)
		 * 0 : unknown error
		 * -1 : time-out (maybe cannot found solution)
		 * -2 : not initialized function
		 * -3 : wrong initial value
		 * -9 : method specific error
		 */
		virtual bool Calculate(double* solutionX, double* solutionY) = 0;
	};
}

#endif