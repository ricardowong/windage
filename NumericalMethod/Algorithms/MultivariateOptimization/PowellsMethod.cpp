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
#include "PowellsMethod.h"

#include "UnimodalOptimization/GoldenSectionSearch2D.h"

using namespace windage;

PowellsMethod::PowellsMethod()
{
	trace = NULL;
	this->dimension = 2;
	params.resize(this->dimension);
}

Vector2 PowellsMethod::FindGradient(Vector2 xk)
{
	const double DISTANCE = 0.1;

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
	Vector2 temp = Vector2(dx, dy);
	return temp /= temp.getLength();
}

double PowellsMethod::FindSetpLength(Vector2 xk, Vector2 pk)
{
	double alpha = 0.0;
	const double CONSTANT1 = 0.6;//1.0e-4;
	const double CONSTANT2 = 0.9;

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

		// wolfe conditions
		alpha = ((1-TAU) * length);
		if(solutionX1 - solutionXk <= FindGradient(xk) * pk * CONSTANT1 * alpha &&
			FindGradient(tempX1) * pk >= FindGradient(xk) * pk * CONSTANT2)
		{
			return alpha;
		}
		alpha = (TAU * length);
		if(solutionX2 - solutionXk <= FindGradient(xk) * pk * CONSTANT1 * alpha &&
			FindGradient(tempX2) * pk >= FindGradient(xk) * pk * CONSTANT2)
		{
			return alpha;
		}
	}

	return 0.0;
}

Vector2 SetReturnValue2(std::vector<Vector2>* point)
{
	const int N = point->size();

	Vector2 temp = Vector2();
	for(int i=0; i<N; i++)
	{
		temp += (*point)[i];
	}
	temp /= (double)(N);
	return temp;
}


bool PowellsMethod::Calculate(double* solutionX, double* solutionY)
{
	const int N = this->dimension;
	this->repeat = 1;

	// initialize
	std::vector<Vector2> point;		point.resize(N + 1);
	std::vector<Vector2> direction; direction.resize(N);

	for(int i=0; i<N; i++)
	{
		Vector2 temp;
		temp.v[i] = 1.0;
		direction[i] = temp;
	}

	srand(time(NULL));
	double x = initialPosition.x;
	double y = initialPosition.y;
	if(x == 0 && y == 0)
	{
		x = (double)(rand() - RAND_MAX/2);
		y = (double)(rand() - RAND_MAX/2);
	}
	Vector2 xi = Vector2(x, y);

	// processing
	int index = 0;
	bool processing = true;
	while(processing)
	{
		double GAMMA = 1.0;

		// s1Me
		point[0] = xi;

		// s2
		for(int k=1; k<N+1; k++)
		{
			// set descent direction
			Vector2 grad = FindGradient(point[k-1]);
			if(direction[k-1] * grad >= 0)
				direction[k-1] = -direction[k-1];

			// find step GAMMA
			GAMMA = FindSetpLength(point[k-1], direction[k-1]);
			if(GAMMA < LEAST_ERROR_RANGE)
			{
				(*solutionX) = point[0].x;
				(*solutionY) = point[0].y;
				return true;
			}

			Vector2 temp = point[k-1] + direction[k-1] * GAMMA;
			point[k] = temp;
		}

		// s4
		for(int j=0; j<N-1; j++)
		{
			direction[j] = direction[j+1];
		}
		direction[N-1] = point[N] - point[0];
		direction[N-1] /= direction[N-1].getLength();

		// s5
		// find step GAMMA
		GAMMA = FindSetpLength(point[0], direction[N-1]);
		if(GAMMA < LEAST_ERROR_RANGE)
		{
			(*solutionX) = point[0].x;
			(*solutionY) = point[0].y;
			return true;
		}

		xi = point[0] + direction[N-1] * GAMMA;

//*
		if(trace)
		{
			Vector2 temp = SetReturnValue2(&point);
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
			Vector2 temp = SetReturnValue2(&point);
			(*solutionX) = temp.x;
			(*solutionY) = temp.y;
			return false;
//*/
		}
	}

	return true;
}