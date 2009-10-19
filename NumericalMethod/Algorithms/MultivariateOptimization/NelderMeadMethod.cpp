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
#include "NelderMeadMethod.h"

using namespace windage;

bool SortEvaluationNPoint(std::vector<double>* evaluation, std::vector<Vector2>* point)
{
	int size = evaluation->size();

	for(int i=size-1; i>0; i--)
	{
		int index = 0;
		double max = 1.0e-10;
		for(int j=0; j<=i; j++)
		{
			if(max < (*evaluation)[j])
			{
				index = j;
				max = (*evaluation)[j];
			}
		}

		//
		double tempE = (*evaluation)[index];
		Vector2 tempP = (*point)[index];

		(*evaluation)[index] = (*evaluation)[i];
		(*point)[index] = (*point)[i];
		(*evaluation)[i] = tempE;
		(*point)[i] = tempP;
	}

	return true;
}

int NelderMeadMethod::Reflection(std::vector<double>* evaluation, std::vector<Vector2>* point)
{
	const int N = this->dimension;

	Vector2 c = Vector2();
	for(int i=0; i<N; i++)
	{
		c += (*point)[i];
	}
	c /= (double)N;

	Vector2 xn1 = (*point)[N];
	Vector2 xr = c + ( c - xn1 ) * this->ALPHA;

	params[0] = xr.x; params[1] = xr.y;
	double fr = function(&params, 2);
	double fn = (*evaluation)[N-1];

	if((*evaluation)[0] <= fr && fr <= fn)
	{
		(*point)[N] = xr;
		(*evaluation)[N] = fr;
		return 1;
	}
	if(fr >= fn)
	{
		return 3;
	}
	if(fr < (*evaluation)[0])
	{
		return 2;
	}

	return 0;
}

int NelderMeadMethod::Expansion(std::vector<double>* evaluation, std::vector<Vector2>* point)
{
	const int N = this->dimension;

	Vector2 c = Vector2();
	for(int i=0; i<N; i++)
	{
		c += (*point)[i];
	}
	c /= (double)N;

	Vector2 xn1 = (*point)[N];
	Vector2 xr = c + ( c - xn1 ) * this->ALPHA;
	params[0] = xr.x; params[1] = xr.y;
	double fr = function(&params, 2);

	Vector2 xe = c + ( xr - c ) * this->BETA;
	params[0] = xe.x; params[1] = xe.y;
	double fe = function(&params, 2);
	
	if(fe <= fr)
	{
		(*point)[N] = xe;
		(*evaluation)[N] = fe;
	}
	else
	{
		(*point)[N] = xr;
		(*evaluation)[N] = fr;
	}

	return 1;
}

int NelderMeadMethod::Contraction(std::vector<double>* evaluation, std::vector<Vector2>* point)
{
	const int N = this->dimension;

	Vector2 c = Vector2();
	for(int i=0; i<N; i++)
	{
		c += (*point)[i];
	}
	c /= (double)N;

	Vector2 xn1 = (*point)[N];
	Vector2 xr = c + ( c - xn1 ) * this->ALPHA;
	double fn1 = (*evaluation)[N];
	params[0] = xr.x; params[1] = xr.y;
	double fr = function(&params, 2);

	Vector2 xc;
	if(fr < fn1)
	{
		xc = c + (xr - c) * this->GAMMA;
	}
	else
	{
		xc = c + (xn1 - c) * this->GAMMA;
	}
	params[0] = xc.x; params[1] = xc.y;
	double fc = function(&params, 2);

	if(fc < MIN(fr, (*evaluation)[N]))
	{
		(*point)[N] = xc;
	}
	else
	{
		Vector2 x1 = (*point)[0];
		for(int i=0; i<N; i++)
		{
			(*point)[i] = (x1 + (*point)[i]) / 2.0;
			params[0] = (*point)[i].x; params[1] = (*point)[i].y;
			(*evaluation)[i] = function(&params, 2);
		}
	}

	return 1;
}

Vector2 SetReturnValue(std::vector<Vector2>* point)
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

bool NelderMeadMethod::Calculate(double* solutionX, double* solutionY)
{
	const int N = this->dimension;
	this->repeat = 1;

	// select randomly
	std::vector<Vector2> point;		point.resize(N + 1);
	std::vector<double> evaluation; evaluation.resize(N + 1);

	srand(time(NULL));
	for(int i=0; i<N+1; i++)
	{
		double x = (double)(rand() - RAND_MAX/2);
		double y = (double)(rand() - RAND_MAX/2);

		point[i] = Vector2(x, y);
		params[0] = x; params[1] = y;
		evaluation[i] = function(&params, 2);
	}
	
	// processing
	int nextStep = 1;
	bool processing = true;
	while(processing)
	{
		// logging data
		if(trace)
		{
			trace->log("step", nextStep);
			std::vector<double> tempX;
			tempX.resize(2);
			for(int i=0; i<point.size(); i++)
			{
				tempX[0] = point[i].x;
				tempX[1] = point[i].y;
				double z = function(&tempX, 2);

				trace->log("x", point[i].x);
				trace->log("y", point[i].y);
				trace->log("z", z);
			}
			trace->logNewLine();
		}

		switch(nextStep)
		{
		case 1:
			SortEvaluationNPoint(&evaluation, &point);
			nextStep = Reflection(&evaluation, &point);
			break;
		case 2:
			nextStep = Expansion(&evaluation, &point);
			break;
		case 3:
			nextStep = Contraction(&evaluation, &point);
			break;
		}

		// terminate?
		double distance = 0.0;
		for(int i=0; i<N; i++)
		{
			distance += point[i].getDistance(point[i+1]);
		}
		distance /= (double)N;

		if(distance < LEAST_ERROR_RANGE)
		{
			Vector2 temp = SetReturnValue(&point);
			(*solutionX) = temp.x;
			(*solutionY) = temp.y;
			return true;
		}

		this->repeat++;
		if(this->repeat > MAX_INTERATE_TIME)
		{
			Vector2 temp = SetReturnValue(&point);
			(*solutionX) = temp.x;
			(*solutionY) = temp.y;
			return false;
		}
	}

	return true;
}