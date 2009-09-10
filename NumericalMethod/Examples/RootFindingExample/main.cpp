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

#include <iostream>
#include <string.h>

#include "RootFinding.h"
#include "BisectionMethod.h"
#include "NewtonMethod.h"
#include "SecantMethod.h"
#include "FalsiMethod.h"

#include "Logger.h"

#define FUNCTION function_example2
#define DERIVATIVE_FUNCTION derivative_function_example2
const long double INITIAL_VALUE_1 = -2.0;
const long double INITIAL_VALUE_2 = +2.0;

// problem set
long double function_example1(long double x)
{
	return pow(x, 3) + 3*x + 1 ;
}
long double derivative_function_example1(long double x)
{
	return 3*pow(x, 2) + 3;
}

// no-solution
long double function_example2(long double x)
{
	return pow(x, 4) + 3*pow(x, 2) + 1;
}
long double derivative_function_example2(long double x)
{
	return 4*pow(x, 3) + 6*x;
}

long double function_example3(long double x)
{
	return -pow(x, 5) - 37*pow(x, 4) + 7*pow(x, 3) - 290*pow(x, 2) + 3*x + 1;
}
long double derivative_function_example3(long double x)
{
	return -5*pow(x, 4) - 148*pow(x, 3) + 21*x*pow(x, 2) - 580*x + 3;
}

long double function_example4(long double x)
{
	return 7*sin(x) - 5*pow(x, 5) + 7;
}
long double derivative_function_example4(long double x)
{
	return 7*cos(x) - 25*pow(x, 4);
}

long double function_example5(long double x)
{
	return function_example1(x) * function_example3(x);
}
long double derivative_function_example5(long double x)
{
	return derivative_function_example1(x) * function_example3(x) + derivative_function_example3(x) * function_example1(x);
}

std::string PrintErrorMessage(double value)
{
	switch((int)value)
	{
//	case 0: return std::string("unknown error");break;
//	case -1:return std::string("time-out (maybe cannot found solution)");break;
	case -2:return std::string("not initialized function");break;
	case -3:return std::string("wrong initial value");break;
	case -9:return std::string("method specific error");break;
	default:
		{
			char message[100];
			sprintf(message, "time-out ( %lf : %e )", value, FUNCTION(value));
			return std::string(message);break;
		}
	}
}

void main()
{
	windage::Logger* logger = new windage::Logger(&std::cout);

	std::cout << "root-finding algorithm examples\n" << std::endl;
	long double solution = 0.0;

	/** BisectionMethod */
	windage::BisectionMethod* bisectionMethod = new windage::BisectionMethod();
//*
	std::cout << "[[ BisectionMethod ]]" << std::endl;
	bisectionMethod->AttatchFunction(FUNCTION);
	bisectionMethod->SetInitialValue(INITIAL_VALUE_1, INITIAL_VALUE_2);
	
	logger->updateTickCount();
	if(bisectionMethod->Calculate(&solution))
		std::cout << "\tcalculate root-finding : " << solution << " (repeat : " << bisectionMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found root-finding" << " (repeat : " << bisectionMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solution).c_str() << ")" << std::endl;
	}
	logger->log("\tprocessing time", logger->calculateProcessTime());
	logger->logNewLine();
//*/
	/** NewtonMethod */
	windage::NewtonMethod* newtonMethod = new windage::NewtonMethod();

	std::cout << "[[ NewtonMethod ]]" << std::endl;
	newtonMethod->AttatchFunction(FUNCTION);
	newtonMethod->AttatchDerivativeFunction(DERIVATIVE_FUNCTION);
	newtonMethod->SetInitialValue(INITIAL_VALUE_1);

	logger->updateTickCount();
	if(newtonMethod->Calculate(&solution))
		std::cout << "\tcalculate root-finding : " << solution << " (repeat : " << newtonMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found root-finding" << " (repeat : " << newtonMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solution).c_str() << ")" << std::endl;
	}
	logger->log("\tprocessing time", logger->calculateProcessTime());
	logger->logNewLine();

	/** SecantMethod */
	windage::SecantMethod* secantMethod = new windage::SecantMethod();
//*	
	std::cout << "[[ SecantMethod ]]" << std::endl;
	secantMethod->AttatchFunction(FUNCTION);
	secantMethod->SetInitialValue(INITIAL_VALUE_1, INITIAL_VALUE_2);

	logger->updateTickCount();
	if(secantMethod->Calculate(&solution))
		std::cout << "\tcalculate root-finding : " << solution << " (repeat : " << secantMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found root-finding" << " (repeat : " << secantMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solution).c_str() << ")" << std::endl;
	}
	logger->log("\tprocessing time", logger->calculateProcessTime());
	logger->logNewLine();
//*/
	/** FalsiMethod */
	windage::FalsiMethod* falsiMethod = new windage::FalsiMethod();

	std::cout << "[[ FalsiMethod ]]" << std::endl;
	falsiMethod->AttatchFunction(FUNCTION);
	falsiMethod->SetInitialValue(INITIAL_VALUE_1, INITIAL_VALUE_2);

	logger->updateTickCount();
	if(falsiMethod->Calculate(&solution))
		std::cout << "\tcalculate root-finding : " << solution << " (repeat : " << falsiMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found root-finding" << " (repeat : " << falsiMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solution).c_str() << " )" << std::endl;
	}
	logger->log("\tprocessing time", logger->calculateProcessTime());
	logger->logNewLine();
}