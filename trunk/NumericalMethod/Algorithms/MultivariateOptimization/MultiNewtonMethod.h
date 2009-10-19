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

#ifndef _MULTI_NEWTON_METHOD_H_
#define _MULTI_NEWTON_METHOD_H_

#include "MultivariateOptimization.h"
#include "Utils/wVector.h"
#include "Utils/wMatrix.h"

#include <vector>

namespace windage
{
	class MultiNewtonMethod : public MultivariateOptimization
	{
	private:
		int dimension;

		Matrix2 FindHessian(Vector2 xk);
		Vector2 FindGradient(Vector2 xk);
		double FindSetpLength(Vector2 xk, Vector2 pk);

		Vector2 initialPosition;
		std::vector<double> params;

		function_pointer functionDx;
		function_pointer functionDy;
		function_pointer functionDx2;
		function_pointer functionDy2;
		function_pointer functionDxDy;
		function_pointer functionDyDx;

	public:
		MultiNewtonMethod();
		inline ~MultiNewtonMethod(){};

		inline void SetDemesion(int dimension){this->dimension = dimension;};
		inline void SetInitialPosition(Vector2 position){this->initialPosition = position;};
		bool Calculate(double* solutionX, double* solutionY);

		inline void AttatchDerivativeFunctionDx(function_pointer function){this->functionDx = function;};
		inline void AttatchDerivativeFunctionDy(function_pointer function){this->functionDy = function;};
		inline void AttatchDerivativeFunctionDx2(function_pointer function){this->functionDx2 = function;};
		inline void AttatchDerivativeFunctionDy2(function_pointer function){this->functionDy2 = function;};
		inline void AttatchDerivativeFunctionDxDy(function_pointer function){this->functionDxDy = function;};
		inline void AttatchDerivativeFunctionDyDx(function_pointer function){this->functionDyDx = function;};
	};
}

#endif