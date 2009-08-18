#ifndef _SPATIAL_SENSOR_H_
#define _SPATIAL_SENSOR_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Utils/wVector.h"

namespace windage
{
	class DLLEXPORT SpatialSensor
	{
	protected:
		Vector3 position;
		bool activation;

		inline void SetActivation(bool state){this->activation = state;};

	public:
		SpatialSensor();
		virtual ~SpatialSensor();

		inline void SetPosition(Vector3 position){this->position = position;};
		inline Vector3 GetPosition(){return this->position;};
		inline bool IsActive(){return this->activation;};
		
//		virtual IsActive()=0;
	};
}


#endif