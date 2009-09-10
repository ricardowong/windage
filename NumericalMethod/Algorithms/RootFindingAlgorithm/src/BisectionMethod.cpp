#include "BisectionMethod.h"

using namespace windage;
BisectionMethod::BisectionMethod()
{
	this->xMin = -1.0;
	this->xMax = 1.0;
}

bool BisectionMethod::Calculate(double* solution)
{
	this->repeat = 0;

	// fault initialization
	if(!function)
		return false;

	double localXMin = this->xMin;
	double localXMax = this->xMax;

	
	double minResult = function(localXMin);
	double maxResult = function(localXMax);

	// fault initial value
	if(minResult * maxResult > 0)
		return false;

	// calculate
	bool processing = true;
	while(processing)
	{
		if(minResult == 0)
		{
			(*solution) = localXMin;
			return true;
		}
		if(minResult == 0)
		{
			(*solution) = localXMax;
			return true;
		}

		minResult = function(localXMin);
		maxResult = function(localXMax);
		
		double mid = (localXMin + localXMax) / 2.0;
		double midResult = function(mid);

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

		if(localXMax-localXMin < 0.000000001)
		{
			(*solution) = localXMin;
			return true;
		}

		this->repeat++;
	}

	return false;
}