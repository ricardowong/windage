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

#include <time.h>

#include "Logger.h"
using namespace windage;

Logger::Logger()
{
	char filename[100];
	sprintf(filename, "log_%s.txt", Logger::getTimeString());
	logging = new std::ofstream(filename);

	fileStream = true;
}

Logger::Logger(char* filenameString, bool addTime)
{
	char filename[100];
	if(addTime)
		sprintf(filename, "%s_%s.txt", filenameString, Logger::getTimeString());
	else
		sprintf(filename, "%s", filenameString);
	logging = new std::ofstream(filename);
	fileStream = true;
}

Logger::Logger(std::string filenameString, bool addTime)
{
	time_t now;
	time(&now);
	struct tm *t;
	t = localtime(&now);

	char filename[100];
	if(addTime)
		sprintf(filename, "%s_%d-%d-%d_%d_%d_%d.txt", filenameString.c_str(), t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	logging = new std::ofstream(filename);
	fileStream = true;
}

Logger::Logger(std::ostream* out)
{
	logging = out;
	fileStream = false;
}

Logger::~Logger()
{
	if(fileStream)
		((std::ofstream*)logging)->close();
}

void Logger::logNewLine()
{
	(*logging) << std::endl;
}

void Logger::log(char* data)
{
	(*logging) << data << " ";
}
void Logger::log(char* dataName, char data)
{
	(*logging) << dataName << "=" << data << " ";
}

void Logger::log(char* dataName, int data)
{
	(*logging) << dataName << "=" << data << " ";
}

void Logger::log(char* dataName, double data)
{
	(*logging) << dataName << "=" << data << " ";
}

void Logger::log(char* dataName, float data)
{
	(*logging) << dataName << "=" << data << " ";
}

void Logger::updateTickCount()
{
	tickCount = cvGetTickCount();
}

double Logger::calculateProcessTime()
{
	processTime = ((cvGetTickCount() - tickCount)/cvGetTickFrequency())/1000.;
	return processTime;
}

double Logger::calculateFPS()
{
	processTime = 1000000./(((cvGetTickCount() - tickCount)/cvGetTickFrequency()));
	return processTime;
}

std::string Logger::getTimeString()
{
	time_t now;
	time(&now);
	struct tm *t;
	t = localtime(&now);

	char timestemp[100];
	sprintf(timestemp, "%d-%d-%d_%d_%d_%d", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

	std::string result = timestemp;
	return result;
}