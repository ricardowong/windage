#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <fstream>
#include <string>

#include <cv.h>

namespace windage
{

	class Logger
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
