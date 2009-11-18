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

#ifndef _SENSOR_GROUP_H_
#define _SENSOR_GROUP_H_

#include <vector>
#include <cv.h>

#include "base.h"
#include "Utils/wVector.h"
#include "SpatialInteraction/SpatialSensor.h"

namespace windage
{
	/**
	 * @brief
	 *		Abstract Class for Spatial Sensor Group
	 * @author
	 *		windage
	 */
	class DLLEXPORT SensorGroup
	{
	protected:
		int id;				///< sensor group id
		Vector3 position;	///< sensor group position
		Vector3 rotation;	///< sensor group rotation

		std::vector<SpatialSensor*> sensors;	///< spatial sensor repository

		void Release();

	public:
		SensorGroup();
		virtual ~SensorGroup();

		/**
		 * @brief
		 *		Set Sensor Group Position
		 * @remark
		 *		set sensor group position and update sensor position
		 */
		void SetPosition(Vector3 position);

		/**
		 * @brief
		 *		Set Sensor Group Rotation
		 * @remark
		 *		set sensor group rotation and update sensor position
		 * @warning
		 *		currently do not operate
		 */
		void SetRotation(Vector3 rotation);

		inline void SetID(int id){this->id = id;};
		inline int GetID(){return id;};
		inline int GetSensorCount(){return (int)sensors.size();};
		inline Vector3 GetPosition(){return position;};
		inline Vector3 GetRotation(){return rotation;};

		inline std::vector<SpatialSensor*>* GetSensors(){return &sensors;};

		/**
		 * @brief
		 *		Add Sensor
		 * @remark
		 *		add sensor to sensor group
		 * @warning
		 *		currently do not apply sensor group rotation information
		 *		and do not remove spatial sensor data at outside
		 */
		void AddSensor(SpatialSensor* sensor, bool absolute=false);
	};
}


#endif