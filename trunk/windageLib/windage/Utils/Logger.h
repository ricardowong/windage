/* ========================================================================
 * PROJECT: windage Library
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

#ifndef _LOGGER_H_
#define _LOGGER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <fstream>
#include <string>

#include <cv.h>

namespace windage
{
	class DLLEXPORT Logger
	{
	public:
		Logger();
		Logger(char* filenameString, bool addTime=false);
		Logger(std::string filenameString, bool addTime=false);
		Logger(std::ostream* out);
		~Logger();
		void logNewLine();
		void log(char* data);
		void log(char* dataName, char data);
		void log(char* dataName, int data);
		void log(char* dataName, double data);
		void log(char* dataName, float data);

		void updateTickCount();
		double calculateProcessTime();
		double calculateFPS();
		inline double getProcessTime(){return processTime;};

	private:
		std::ostream* logging;
		bool fileStream;
		double tickCount;
		double processTime;
	};
}


#endif
