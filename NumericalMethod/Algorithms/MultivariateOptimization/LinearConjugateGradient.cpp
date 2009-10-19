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

#include <time.h>
#include "LinearConjugateGradient.h"

#include "UnimodalOptimization/GoldenSectionSearch2D.h"

using namespace windage;
LinearConjugateGradient::LinearConjugateGradient()
{
	trace = NULL;
	this->dimension = 2;
	params.resize(this->dimension);

	initialPosition = windage::Vector2();
}

bool LinearConjugateGradient::Calculate(double* solutionX, double* solutionY)
{
	Vector2 pk;
	Vector2 rk;
	Vector2 point;
	double ak;

	double x = initialPosition.x;
	double y = initialPosition.y;
	if(x == 0 && y == 0)
	{
		srand(time(NULL));
		x = (double)(rand() - RAND_MAX/2);
		y = (double)(rand() - RAND_MAX/2);
	}
	point = Vector2(x, y);

	rk = A * point - b;
	pk = -rk;

	// processing
	int index = 0;
	bool processing = true;
	while(processing)
	{
		if(rk.getLength() < LEAST_ERROR_RANGE)
		{
			(*solutionX) = point.x;
			(*solutionY) = point.y;
			return true;
		}

		ak = -(rk * pk) / (A * pk * pk);
		point = point + pk * ak;
		Vector2 rk1 = rk + A * pk * ak;
		double Bk = (rk1 * rk1) / (rk * rk);
		pk = -rk1 + pk * Bk;

		rk = rk1;

//*
		if(trace)
		{
			Vector2 temp = point;
			std::vector<double> tempX;
			tempX.push_back(temp.x);
			tempX.push_back(temp.y);
			double z = function(&tempX, 2);
			
			trace->log("repeat", this->repeat);
			trace->log("x", temp.x);
			trace->log("y", temp.y);
			trace->log("z", z);
			trace->logNewLine();
		}
//*/

		this->repeat++;
		if(this->repeat > MAX_INTERATE_TIME)
		{
			Vector2 temp = point;
			(*solutionX) = temp.x;
			(*solutionY) = temp.y;
			return false;
//*/
		}
	}

	return false;
}