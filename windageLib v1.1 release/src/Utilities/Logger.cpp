/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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

#include "Utilities/Logger.h"
using namespace windage;

void Logger::logNewLine()
{
	(*logging) << std::endl;
}

void Logger::log(char* dataName, char* data)
{
	(*logging) << dataName << EQUAL_TERM;
	log(data);
}

void Logger::log(char* dataName, char data)
{
	(*logging) << dataName << EQUAL_TERM;
	log(data);
}

void Logger::log(char* dataName, int data)
{
	(*logging) << dataName << EQUAL_TERM;
	log(data);
}

void Logger::log(char* dataName, double data)
{
	(*logging) << dataName << EQUAL_TERM;
	log(data);
}

void Logger::log(char* dataName, float data)
{
	(*logging) << dataName << EQUAL_TERM;
	log(data);
}

void Logger::log(const char* data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(char* data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(char data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(int data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(double data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(float data)
{
	(*logging) << data << SPACING_TERM;
}

void Logger::log(windage::Vector2 data)
{
	(*logging) << data.x << SPACING_TERM << data.y << SPACING_TERM;
}

void Logger::log(windage::Vector3 data)
{
	(*logging) << data.x << SPACING_TERM << data.y << SPACING_TERM << data.z << SPACING_TERM;
}

void Logger::log(windage::Vector4 data)
{
	(*logging) << data.x << SPACING_TERM << data.y << SPACING_TERM << data.z << SPACING_TERM << data.w << SPACING_TERM;
}

void Logger::log(windage::Matrix2 data)
{
	int size = 2;
	for(int y=0; y<size; y++)
	{
		for(int x=0; x<size; x++)
		{
			log(data.m[y][x]);
		}
		logNewLine();
	}
}

void Logger::log(windage::Matrix3 data)
{
	int size = 3;
	for(int y=0; y<size; y++)
	{
		for(int x=0; x<size; x++)
		{
			log(data.m[y][x]);
		}
		logNewLine();
	}
}

void Logger::log(windage::Matrix4 data)
{
	int size = 4;
	for(int y=0; y<size; y++)
	{
		for(int x=0; x<size; x++)
		{
			log(data.m[y][x]);
		}
		logNewLine();
	}
}

void Logger::log(CvScalar data)
{
	(*logging) << data.val[0] << SPACING_TERM << data.val[1] << SPACING_TERM << data.val[2] << SPACING_TERM << data.val[3] << SPACING_TERM;
}

void Logger::log(CvMat* data)
{
	int row = data->rows;
	int column = data->cols;
	for(int y=0; y<row; y++)
	{
		for(int x=0; x<column; x++)
		{
			log(CV_MAT_ELEM((*data), double, y, x));
		}
		logNewLine();
	}
}

void Logger::updateTickCount()
{
	tickCount = (double)cvGetTickCount();
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
	struct tm t;
	localtime_s(&t, &now);

	char timestemp[100];
	sprintf_s(timestemp, "%d-%02d-%02d_%02d_%02d_%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);

	std::string result = std::string(timestemp);
	return result;
}