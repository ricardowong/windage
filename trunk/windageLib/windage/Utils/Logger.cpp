#include <time.h>

#include "Logger.h"
using namespace windage;

Logger::Logger()
{
	time_t now;
	time(&now);
	struct tm *t;
	t = localtime(&now);

	char filename[100];
	sprintf(filename, "log_%d-%d-%d_%d_%d_%d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	logging = new std::ofstream(filename);

	fileStream = true;
}

Logger::Logger(char* filenameString, bool addTime)
{
	time_t now;
	time(&now);
	struct tm *t;
	t = localtime(&now);

	char filename[100];
	if(addTime)
		sprintf(filename, "%s_%d-%d-%d_%d_%d_%d.txt", filenameString, t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
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
	processTime = 1000./(((cvGetTickCount() - tickCount)/cvGetTickFrequency())/1000.);
	return processTime;
}