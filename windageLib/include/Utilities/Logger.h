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

/**
 * @file	Logger.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is log class for logging at processtime and programmer's message
 */

#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <fstream>
#include <string>

#include <cv.h>

#include "base.h"
#include "Structures/Matrix.h"

namespace windage
{
	/**
	 * @defgroup Utilities Utility classes
	 * @brief
	 *		Utility classes
	 * @addtogroup Utilities
	 * @{
	 */

	/**
	 * @brief	Class for Logging at processtime and programmer's message
	 * @author	Woonhyuk Baek
	 */
	class DLLEXPORT Logger
	{
	private:
		char SPACING_TERM;
		char EQUAL_TERM;

		std::ostream* logging;	///< logging target (file or std::cout stream)
		bool fileStream;
		double tickCount;
		double processTime;

		char filename[100];

	public:
		Logger(char* filenameString="log", char* extention="txt", bool addTime=false)
		{
			SPACING_TERM = ' ';
			EQUAL_TERM = '=';

			if(addTime)
				sprintf_s(filename, "%s_%s.%s", filenameString, Logger::getTimeString().c_str(), extention);
			else
				sprintf_s(filename, "%s.%s", filenameString, extention);
			logging = new std::ofstream(filename);
			fileStream = true;
		}
		Logger(std::ostream* out)
		{
			SPACING_TERM = ' ';
			EQUAL_TERM = '=';

			logging = out;
			fileStream = false;
		}
		~Logger()
		{
			if(fileStream)
				((std::ofstream*)logging)->close();
		}

		char* GetFilename(){return this->filename;}

		/**
		 * @defgroup logging user message & data
		 * @brief
		 *		User Message Logging Method
		 * @remark
		 *		logging "dataName=data" or "data"
		 * @addtogroup Logging
		 * @{
		 */
		void logNewLine();
		void log(char* dataName, char* data);
		void log(char* dataName, char data);
		void log(char* dataName, int data);
		void log(char* dataName, double data);
		void log(char* dataName, float data);
		void log(const char* data);
		void log(char* data);
		void log(char data);
		void log(int data);
		void log(double data);
		void log(float data);
		void log(windage::Vector2 data);
		void log(windage::Vector3 data);
		void log(windage::Vector4 data);
		void log(windage::Matrix2 data);
		void log(windage::Matrix3 data);
		void log(windage::Matrix4 data);
		void log(CvScalar data);
		void log(CvMat* data);
		/** @} */

		/**
		 * @fn	updateTickCount
		 * @brief
		 *		update Tick Count
		 * @remark
		 *		update tick count member value from update currnet tick count
		 */
		void updateTickCount();
		inline double getProcessTime(){return processTime;};

		/**
		 * @fn	calculateProcessTime
		 * @brief
		 *		calcuate processing time
		 * @remark
		 *		calcuate processing time after calling updateTickCount method
		 */
		double calculateProcessTime();

		/**
		 * @fn	calculateFPS
		 * @brief
		 *		calcuate FPS
		 * @remark
		 *		calcuate FPS after calling updateTickCount method
		 */
		double calculateFPS();

		/**
		 * @fn	getTimeString
		 * @brief
		 *		get Time Stemp
		 * @remark
		 *		return current time stemp
		 */
		static std::string getTimeString();
	};
	/** @} */ // addtogroup Utilities
}
#endif // _LOGGER_H_