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
#include "NonlinearConjugateGradient.h"

#include "UnimodalOptimization/GoldenSectionSearch2D.h"

using namespace windage;
NonlinearConjugateGradient::NonlinearConjugateGradient()
{
	trace = NULL;
	this->dimension = 2;
	params.resize(this->dimension);

	initialPosition = windage::Vector2();
}

Vector2 NonlinearConjugateGradient::FindGradient(Vector2 xk)
{
	if(functionDx && functionDy)
	{
		params[0] = xk.x;	params[1] = xk.y;
	
		double dx = functionDx(&params, 2);
		double dy = functionDy(&params, 2);
		Vector2 temp = Vector2(dx, dy);
		return temp;// /= temp.getLength();
	}
	else
	{
		const double DISTANCE = DELTA;
		const double DISTANCE2 = DISTANCE * 2;

		params[0] = xk.x - DISTANCE; params[1] = xk.y;
		double x1 = function(&params, 2);
		params[0] = xk.x + DISTANCE; params[1] = xk.y;
		double x2 = function(&params, 2);
		params[0] = xk.x; params[1] = xk.y - DISTANCE;
		double y1 = function(&params, 2);
		params[0] = xk.x; params[1] = xk.y + DISTANCE;
		double y2 = function(&params, 2);

		double dx = x2 - x1;
		double dy = y2 - y1;
		Vector2 temp = Vector2(dx/DISTANCE2, dy/DISTANCE2);
		return temp;// /= temp.getLength();
	}
}

double NonlinearConjugateGradient::FindSetpLength(Vector2 xk, Vector2 pk)
{
	double alpha = 0.0;
	const double CONSTANT1 = 0.0001;//1.0e-4;
	const double CONSTANT2 = 0.3;

	std::vector<double> params;			params.resize(2);
	pk /= pk.getLength();

	params[0] = xk.x; params[1] = xk.y;
	double solutionXk = function(&params, 2);

	Vector2 localXMin = xk;
	Vector2 localXMax = xk + pk * 100.0;
	Vector2 direction = pk;

	double length = (localXMax - localXMin).getLength();
	Vector2 x1 = localXMin + direction * ( (1 - TAU) * length );
	Vector2 x2 = localXMin + direction * ( TAU * length );
	params[0] = x1.x; params[1] = x1.y;
	double solutionX1 = function(&params, 2);
	params[0] = x2.x; params[1] = x2.y;
	double solutionX2 = function(&params, 2);

	for(int i=0; i<1000; i++)
	{
		// calculate
		length = (localXMax - localXMin).getLength();
		Vector2 tempX1 = localXMin + direction * ( (1 - TAU) * length );
		Vector2 tempX2 = localXMin + direction * ( TAU * length );

		if(solutionX1 >= solutionX2)
		{
			localXMin = x1;
		}
		else if(solutionX1 <= solutionX2)
		{
			localXMax = x2;
		}

		x1 = tempX1;
		x2 = tempX2;

		params[0] = x1.x;	params[1] = x1.y;
		solutionX1 = function(&params, 2);
		params[0] = x2.x;	params[1] = x2.y;
		solutionX2 = function(&params, 2);

		// strong wolfe conditions
		alpha = ((1-TAU) * length);
		if(solutionX1 - solutionXk <= FindGradient(xk) * pk * CONSTANT1 * alpha &&
			abs(FindGradient(tempX1) * pk) <= - FindGradient(xk) * pk * CONSTANT2)
		{
			return alpha;
		}
		alpha = (TAU * length);
		if(solutionX2 - solutionXk <= FindGradient(xk) * pk * CONSTANT1 * alpha &&
			abs(FindGradient(tempX2) * pk) >= - FindGradient(xk) * pk * CONSTANT2)
		{
			return alpha;
		}
	}

	return 0.0;
}

bool NonlinearConjugateGradient::Calculate(double* solutionX, double* solutionY)
{
	Vector2 pk;
	Vector2 fk;
	Vector2 point;
	double ak = 1.0;

	double x = initialPosition.x;
	double y = initialPosition.y;
	if(x == 0 && y == 0)
	{
		srand(time(NULL));
		x = (double)(rand() - RAND_MAX/2);
		y = (double)(rand() - RAND_MAX/2);
	}
	point = Vector2(x, y);

	fk = FindGradient(point);
	pk = -fk;

	// processing
	int index = 0;
	bool processing = true;
	while(processing)
	{
		if(fk.getLength() < LEAST_ERROR_RANGE || ak < LEAST_ERROR_RANGE)
		{
			(*solutionX) = point.x;
			(*solutionY) = point.y;
			return true;
		}

		pk /= pk.getLength();
		ak = FindSetpLength(point, pk);
		point = point + pk * ak;
		Vector2 fk1 = FindGradient(point);

		// CG-FR (Flectcher-Reeves)
		double Bk = (fk1 * fk1) / (fk * fk);
		pk = -fk1 + pk * Bk;

		fk = fk1;

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