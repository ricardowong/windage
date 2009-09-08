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

#ifndef _SURF_FEATURE_H_
#define _SURF_FEATURE_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>
#include "Utils/wMatrix.h"

namespace windage
{
	const int SURF_DESCRIPTOR_DIMENSION = 36;	///< Modified SURF Descriptor dimension = 36 (fixed)
	typedef struct _Description
	{
		double descriptor[SURF_DESCRIPTOR_DIMENSION];	///< SURF Descriptor 36-dimension
		struct _Description()
		{
			for(int i=0; i<SURF_DESCRIPTOR_DIMENSION; i++)
				descriptor[i] = 0.0;
		}
		void operator=(struct _Description oprd)
		{
			for(int i=0; i<SURF_DESCRIPTOR_DIMENSION; i++)
				descriptor[i] = oprd.descriptor[i];
		}
		double distance(struct _Description oprd)
		{
			double sum = 0;
			for(int i=0; i<SURF_DESCRIPTOR_DIMENSION; i++)
				sum += (double)((descriptor[i] - oprd.descriptor[i]) * (descriptor[i] - oprd.descriptor[i]));
			return sum;
		}
	}Description;

	class DLLEXPORT SURFFeature
	{
	private:
		const static int PATCH_WIDTH = 45;
		const static int PATCH_HEIGHT = 45;

		Vector3 position;
		IplImage* patch;
		std::vector<Description> descriptionList;

		double scaleFactor;
		int scaleStep;

	public:
		SURFFeature();
		~SURFFeature();

		inline int GetPatchWidth(){return this->PATCH_WIDTH;};
		inline int GetPatchHeight(){return this->PATCH_HEIGHT;};
		inline Vector3 GetPosition(){return this->position;};
		inline void SetPosition(Vector3 position){this->position = position;};
		inline IplImage* GetPatch(){return this->patch;};

		inline int GetFeatureCount(){return this->descriptionList.size();};
		inline std::vector<Description>* GetDescriptionList(){return &this->descriptionList;};

		void Release();
		void initialize(double scaleFactor=3.0, int scaleStep=6);
		int GenerateDescriptor(IplImage* grayImage, CvPoint point);
		int ExtractModifiedSURF(IplImage* grayImage, CvPoint center, Description* descriptions);
		Description GetDescription(int index);
	};
}

#endif