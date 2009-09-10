#include <iostream>

#include "RootFinding.h"
#include "BisectionMethod.h"

double function_example1(double x)
{
	return x*x*x + 3*x + 1 ;
}
double function_example2(double x)
{
	return x*x*x*x*x + 3*x - 1;
}

void main()
{
	std::cout << "root-finding algorithm examles" << std::endl;

	windage::BisectionMethod* bisectionMethod = new windage::BisectionMethod();
	bisectionMethod->AttatchFunction(function_example2);
	bisectionMethod->SetInitialValue(-1, 1);

	double solution = 0.0;
	if(bisectionMethod->Calculate(&solution))
	{
		std::cout << "calculate root-finding : " << solution << " (repeat : " << bisectionMethod->GetRepatCount() << ")" << std::endl;
	}
	else
	{
		std::cout << "fault : can not found root-finding : " << std::endl;
	}

}