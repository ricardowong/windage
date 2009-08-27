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

#ifndef _SPATIAL_SENSOR_H_
#define _SPATIAL_SENSOR_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>
#include "Utils/wVector.h"

namespace windage
{
	class DLLEXPORT SpatialSensor
	{
	public:
		static enum STATE{INACTIVATE=1, ACTIVATE = 2};

	protected:
		Vector3 position;
		STATE state;

		inline void SetActivation(bool state){if(state) this->state = SpatialSensor::ACTIVATE; else this->state = SpatialSensor::INACTIVATE;};

	public:
		SpatialSensor();
		virtual ~SpatialSensor();

		void Initialize(Vector3 position=Vector3());

		inline void SetPosition(Vector3 position){this->position = position;};
		inline Vector3 GetPosition(){return this->position;};
		inline bool IsActive(){return this->state == SpatialSensor::ACTIVATE;};

		inline void SetActive(){this->state = SpatialSensor::ACTIVATE;};
		inline void SetInactive(){this->state = SpatialSensor::INACTIVATE;};
		inline STATE GetState(){return state;};

//		SpatialSensor& operator=(const SpatialSensor&);
	};
}


#endif