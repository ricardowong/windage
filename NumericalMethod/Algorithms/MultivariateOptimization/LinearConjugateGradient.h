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

#ifndef _LINEAR_CONJUGATE_GRADIENT_METHOD_H_
#define _LINEAR_CONJUGATE_GRADIENT_METHOD_H_

#include "MultivariateOptimization.h"
#include "Utils/wVector.h"
#include "Utils/wMatrix.h"

#include <vector>

namespace windage
{
	class LinearConjugateGradient : public MultivariateOptimization
	{
	private:
		int dimension;
		Vector2 initialPosition;
		std::vector<double> params;

		Matrix2 A;
		Vector2 b;

	public:
		LinearConjugateGradient();
		inline ~LinearConjugateGradient(){};

		inline void SetDemesion(int dimension){this->dimension = dimension;};
		inline void SetInitialPosition(Vector2 position){this->initialPosition = position;};
		inline void SetMatrixA(windage::Matrix2 A){this->A = A;};
		inline void SetVectorB(windage::Vector2 b){this->b = b;};

		bool Calculate(double* solutionX, double* solutionY);
	};
}

#endif