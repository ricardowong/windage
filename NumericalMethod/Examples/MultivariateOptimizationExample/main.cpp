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

#include "MultivariateOptimization/NelderMeadMethod.h"
#include "MultivariateOptimization/PowellsMethod.h"
#include "Utils/Logger.h"

#define FUNCTION function_example1
// problem set
double function_example1(std::vector<double>* x, int N=2)
{
	double xl = (*x)[0];
	double yl = (*x)[1];
	return xl*xl - 2*xl*yl + 2*yl*yl - 6*yl + 9;
}


std::string PrintErrorMessage(double valueX, double valueY)
{
	switch((int)valueX)
	{
//	case 0: return std::string("unknown error");break;
//	case -1:return std::string("time-out (maybe cannot found solution)");break;
	case -2:return std::string("not initialized function");break;
	case -3:return std::string("wrong initial value");break;
	case -9:return std::string("method specific error");break;
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
	windage::Logger* traceNMM = new windage::Logger("trace_NMM.txt");
	windage::Logger* tracePM = new windage::Logger("trace_PM.txt");

	double solutionX = 0.0;
	double solutionY = 0.0;

	windage::NelderMeadMethod* nelderMeadMethod = new windage::NelderMeadMethod();
	nelderMeadMethod->AttatchFunction(FUNCTION);
	nelderMeadMethod->AttatchTrace(traceNMM);

	std::cout << "[[ Nelder Mead Method ]]" << std::endl;
	if(nelderMeadMethod->Calculate(&solutionX, &solutionY))
		std::cout << "\tcalculate local minmum point : " << solutionX << ", " << solutionY << " (repeat : " << nelderMeadMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found local minmum point " << " (repeat : " << nelderMeadMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solutionX, solutionY).c_str() << ")" << std::endl;
	}



	windage::PowellsMethod* powellsMethod = new windage::PowellsMethod();
	powellsMethod->AttatchFunction(FUNCTION);
	powellsMethod->AttatchTrace(tracePM);

	std::cout << "[[ Powell's Method ]]" << std::endl;
	if(powellsMethod->Calculate(&solutionX, &solutionY))
		std::cout << "\tcalculate local minmum point : " << solutionX << ", " << solutionY << " (repeat : " << powellsMethod->GetRepatCount() << ")" << std::endl;
	else
	{
		std::cout << "\tfault : can not found local minmum point " << " (repeat : " << powellsMethod->GetRepatCount() << ")" << std::endl;
		std::cout << "\t\t( " << PrintErrorMessage(solutionX, solutionY).c_str() << ")" << std::endl;
	}

}