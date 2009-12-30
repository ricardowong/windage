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

#ifndef _REFERENCE_STORAGE_H_
#define _REFERENCE_STORAGE_H_

#include <cv.h>

#include "base.h"
#include "FAST/wsurf.h"

namespace windage
{
	class DLLEXPORT SURFReferenceStorage
	{
	private:
		int realWidth;
		int realHeight;
		int featureExtractThreshold;	///< FAST Corner extract threshold value
		IplImage* referenceImage;

		std::vector<SURFDesciription> descriptor;	///< reference image surf description

		CvMat* referenceFeatureStorage;			///< reference feature storage
		cv::Mat flannFeatureTree;
		cv::flann::Index* flannIndex;

		int GenerateReferenceFeatureTree(double scaleFactor=4.0, int scaleStep=8);
		bool CreateFlannTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage);
		void Release();

	public:
		SURFReferenceStorage()
		{
			realWidth = 640;
			realHeight = 480;
			featureExtractThreshold = 30;
			flannIndex = NULL;
			referenceImage = NULL;
			referenceFeatureStorage = NULL;
		}
		~SURFReferenceStorage()
		{
			this->Release();
		}

		inline double GetRealWidth(){return this->realWidth;};
		inline double GetRealHeight(){return this->realHeight;};
		inline int GetFeatureExtractTreshold(){return this->featureExtractThreshold;};
		inline void SetFeatureExtractTreshold(int threshold=30){this->featureExtractThreshold = threshold;};
		inline std::vector<SURFDesciription>* GetDescriptor(){return &this->descriptor;};

		int FindPairs(SURFDesciription description, float distanceRate=0.5, int EMAX=20, float* outDistance = NULL);
		void RegistReferenceImage(	IplImage* referenceImage,	///< reference image
									double realWidth,			///< image real width size
									double realHeight,			///< image real width size
									double scaleFactor=4.0,		///< scale factor for multi-scale
									int scaleStep=8				///< multi-scale step
									);
	};
}


#endif