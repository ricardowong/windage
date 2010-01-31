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

#ifndef _FEATURE_POINT_H_
#define _FEATURE_POINT_H_

#include <vector>

#include <cv.h>
#include "base.h"
#include "Vector.h"

namespace windage
{
	class DLLEXPORT FeaturePoint
	{
	protected:
		windage::Vector3 point;
		int objectID;
		int size;
		double dir;
		double distance;
		bool outlier;

		// to check tracking
		bool tracked;
		int repositoryID;
		
	public:
		int DESCRIPTOR_DIMENSION;
		std::vector<double> descriptor;

	public:
		FeaturePoint()
		{
			this->DESCRIPTOR_DIMENSION = 1;
			this->descriptor.resize(DESCRIPTOR_DIMENSION);

			objectID = -1;
			size = 0;
			dir = 0;
			distance = 1.0e10;
			outlier = false;

			tracked = false;
			repositoryID = -1;
		}
		virtual ~FeaturePoint()
		{
		}

		virtual void operator=(FeaturePoint oprd)
		{
			this->DESCRIPTOR_DIMENSION = oprd.DESCRIPTOR_DIMENSION;
			this->descriptor.resize(DESCRIPTOR_DIMENSION);

			this->point = oprd.GetPoint();
			this->objectID = oprd.GetObjectID();
			this->size = oprd.GetSize();
			this->dir = oprd.GetDir();
			this->distance = oprd.GetDistance();

			this->outlier = oprd.IsOutlier();

			this->tracked = oprd.IsTracked();
			this->repositoryID = oprd.GetRepositoryID();
		}

		virtual double GetDistance(FeaturePoint oprd)
		{
			if(this->DESCRIPTOR_DIMENSION != oprd.DESCRIPTOR_DIMENSION)
				return -1.0;

			double sum = 0.0;
			for(int i=0; i<this->DESCRIPTOR_DIMENSION; i++)
				sum += abs(this->descriptor[i] - oprd.descriptor[i]);
			return sum;
		}

		inline void SetPoint(windage::Vector3 point){this->point = point;};
		inline windage::Vector3 GetPoint(){return this->point;};
		inline void SetObjectID(int id){this->objectID = id;};
		inline int GetObjectID(){return this->objectID;};
		inline void SetSize(int size){this->size = size;};
		inline int GetSize(){return this->size;};
		inline void SetDir(double dir){this->dir = dir;};
		inline double GetDir(){return this->dir;};
		inline void SetDistance(double distance){this->distance = distance;};
		inline double GetDistance(){return this->distance;};
		inline void SetOutlier(bool outlier){this->outlier = outlier;};
		inline bool IsOutlier(){return this->outlier;};

		inline void SetTracked(bool tracked){this->tracked = tracked;};
		inline bool IsTracked(){return this->tracked;};
		inline void SetRepositoryID(int id){this->repositoryID = id;};
		inline int GetRepositoryID(){return this->repositoryID;};
	};
}

#endif