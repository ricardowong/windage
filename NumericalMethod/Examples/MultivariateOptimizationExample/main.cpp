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

#include "MultivariateOptimization/LinearConjugateGradient.h"
#include "MultivariateOptimization/NonlinearConjugateGradient.h"
#include "Utils/Logger.h"

#define FUNCTION		function_example1
#define FUNCTION_DX		function_example1_dx
#define FUNCTION_DY		function_example1_dy
#define INITIAL_POINT windage::Vector2(2.0, -1.0)

// problem set
windage::Matrix2 function1A(2, 2, 2, 4);
windage::Vector2 function1b(1, 1);
double function_example1(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return x1*x1 + 2*x1*y1 + 2*y1*y1 + x1 + y1;
}
double function_example1_dx(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 2*x1 + 2*y1 + 1;
}
double function_example1_dy(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 2*x1 + 4*y1 + 1;
}

double function_example2(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 100 * pow((y1 - x1*x1), 2) + pow((1-x1), 2);
	//return 100*x1*x1*x1*x1 - 200*x1*x1*y1 + 100*y1*y1 + x1*x1 - 2*x1 + 1;
}
double function_example2_dx(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 400*x1*x1*x1 - 400*x1*y1 + 2*x1 - 2;
}
double function_example2_dy(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return - 200*x1*x1 + 200*y1;
}
double function_example2_dx2(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 1200*x1*x1 - 400*y1 + 2;
}
double function_example2_dy2(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return 200;
}
double function_example2_dxdy(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return - 400*x1;
}
double function_example2_dydx(std::vector<double>* x, int N=2)
{
	double x1 = (*x)[0];
	double y1 = (*x)[1];
	return - 400*x1;
}

std::string PrintErrorMessage(double valueX, double valueY)
{
	switch((int)valueX)
	{
//	case 0: return std::string("unknown error");break;
//	case -1:return std::string("time-out (maybe cannot found solution)");break;
//	case -2:return std::string("not initialized function");break;
//	case -3:return std::string("wrong initial value");break;
//	case -9:return std::string("method specific error");break;
	default:
		{
			char message[100];

			std::vector<double> params;
			params.push_back(valueX);
			params.push_back(valueY);
			sprintf(message, "time-out ( %lf, %lf : %lf )", valueX, valueY, FUNCTION(&params, 2));
			return std::string(message);break;
		}
	}
}

void main()
{
	windage::Logger* logger = new windage::Logger(&std::cout);
	windage::Logger* traceLinearCG = new windage::Logger("trace_Linear_CG", true);
	windage::Logger* traceNonlinearCG = new windage::Logger("trace_Nonlinear_CG", true);

	double solutionX = 0.0;
	double solutionY = 0.0;

	windage::LinearConjugateGradient* linearCGMethod = new windage::LinearConjugateGradient();
	linearCGMethod->AttatchFunction(FUNCTION);
	linearCGMethod->SetMatrixA(function1A);
	linearCGMethod->SetVectorB(function1b);
	linearCGMethod->AttatchTrace(traceLinearCG);
	linearCGMethod->SetInitialPosition(INITIAL_POINT);
	
	logger->updateTickCount();
	std::cout << "[[ Linear Conjugate Gradient Method ]]" << std::endl;
	if(linearCGMethod->Calculate(&solutionX, &solutionY))
		std::cout << "\tcalculate local minmum point : " << solutionX << ", " << solutionY << " (repeat : " << linearCGMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found local minmum point " << " (repeat : " << linearCGMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solutionX, solutionY).c_str() << ")" << std::endl;
	}
	logger->log("Linear Conjugate Gradient Method", logger->calculateProcessTime());
	logger->logNewLine();
	logger->logNewLine();


	windage::NonlinearConjugateGradient* nonlinearCGMethod = new windage::NonlinearConjugateGradient();
	nonlinearCGMethod->AttatchFunction(FUNCTION);
	nonlinearCGMethod->AttatchDerivativeFunctionDx(FUNCTION_DX);
	nonlinearCGMethod->AttatchDerivativeFunctionDy(FUNCTION_DY);
	
	nonlinearCGMethod->AttatchTrace(traceNonlinearCG);
	nonlinearCGMethod->SetInitialPosition(INITIAL_POINT);
	
	logger->updateTickCount();
	std::cout << "[[ Nonlinear Conjugate Gradient Method ]]" << std::endl;
	if(nonlinearCGMethod->Calculate(&solutionX, &solutionY))
		std::cout << "\tcalculate local minmum point : " << solutionX << ", " << solutionY << " (repeat : " << nonlinearCGMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found local minmum point " << " (repeat : " << nonlinearCGMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solutionX, solutionY).c_str() << ")" << std::endl;
	}
	logger->log("Nonlinear Conjugate Gradient Method", logger->calculateProcessTime());
	logger->logNewLine();
	logger->logNewLine();

}