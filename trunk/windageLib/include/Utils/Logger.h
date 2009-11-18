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

#include <fstream>
#include <string>

#include <cv.h>

#include "base.h"

namespace windage
{
	/**
	 * @brief
	 *		Class for Logging at processtime and programmer's message
	 * @author
	 *		windage
	 */
	class DLLEXPORT Logger
	{
	public:
		Logger();
		Logger(char* filenameString, bool addTime=false);
		Logger(std::string filenameString, bool addTime=false);
		Logger(std::ostream* out);
		~Logger();

		/**
		 * @defgroup Logging User Message Logging Method
		 * @brief
		 *		User Message Logging Method
		 * @remark
		 *		logging "dataName : data" or "data"
		 * @addtogroup Logging
		 * @{
		 */
		void logNewLine();
		void log(char* data);
		void log(char* dataName, char data);
		void log(char* dataName, int data);
		void log(char* dataName, double data);
		void log(char* dataName, float data);
		/** @} */

		/**
		 * @brief
		 *		update Tick Count
		 * @remark
		 *		update tick count member value from update currnet tick count
		 */
		void updateTickCount();

		/**
		 * @brief
		 *		calcuate processing time
		 * @remark
		 *		calcuate processing time after calling updateTickCount method
		 */
		double calculateProcessTime();

		/**
		 * @brief
		 *		calcuate FPS
		 * @remark
		 *		calcuate FPS after calling updateTickCount method
		 */
		double calculateFPS();
		inline double getProcessTime(){return processTime;};

		/**
		 * @brief
		 *		get Time Stemp
		 * @remark
		 *		return current time stemp
		 */
		static std::string getTimeString();

	private:
		std::ostream* logging;	///< logging target (file or std::cout stream)
		bool fileStream;
		double tickCount;
		double processTime;
	};
}


#endif
