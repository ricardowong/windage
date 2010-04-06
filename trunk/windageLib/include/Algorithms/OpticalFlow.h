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
 * @file	OpticalFlow.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It is implemetation of feature tracking to use Lucas-Kanade Optical flow
 */

#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#include <vector>

#include <cv.h>
#include "base.h"

#include "Structures/FeaturePoint.h"

namespace windage
{
	namespace Algorithms
	{
		/**
		 * @defgroup Algorithms Algorithm classes
		 * @brief
		 *		algorithm classes
		 * @addtogroup Algorithms
		 * @{
		 */

		/**
		 * @defgroup AlgorithmsOpticalFlow Feature Tracking
		 * @brief
				feature tracking algorithm classes
		 * @addtogroup AlgorithmsOpticalFlow
		 * @{
		 */

		/**
		 * @brief	class for feature tracking to use Lucas-Kanade optical flow
		 * @author	Woonhyuk Baek
		 */
		class DLLEXPORT OpticalFlow
		{
		private:
			const static int MAX_POINT_COUNT = 5000;///< initialize maximum value that is the number of feature point

			int imageWidth;							///< input image size
			int imageHeight;						///< input image size

			CvSize windowSize;						///< opticalflow window size
			int pyramidLevel;						///< opticalflow pyramid level
			
			CvPoint2D32f feature1[MAX_POINT_COUNT];	///< preview image feature points
			CvPoint2D32f feature2[MAX_POINT_COUNT];	///< current image feature points
			char foundFeature[MAX_POINT_COUNT];		///< tracking status
			float errorFeature[MAX_POINT_COUNT];

			CvTermCriteria terminationCriteria;		///< terminate criteria
			int eMax;								///< limitation of iteration count
			IplImage* pyramid1;
			IplImage* pyramid2;

			void Release();

		public:
			virtual char* GetFunctionName(){return "OpticalFlow";};
			OpticalFlow(int eMax=20)
			{
				terminationCriteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, eMax, .3);
				pyramid1 = NULL;
				pyramid2 = NULL;
			}
			~OpticalFlow()
			{
				this->Release();
			}

			void SetEMax(int emax)
			{
				this->eMax = emax;
				terminationCriteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, eMax, .3);
			};
			inline int GetEMax(){return this->eMax;};

			inline void SetImageSize(int width, int height){this->imageWidth=width;this->imageHeight=height;};
			inline CvSize GetImageSize(){return cvSize(this->imageWidth, this->imageHeight);};
			inline void SetWindowSize(CvSize size=cvSize(8, 8)){this->windowSize = size;};
			inline CvSize GetWindowSize(){return this->windowSize;};
			inline void SetPyramidLevel(int level=3){this->pyramidLevel = level;};
			inline int GetPyramidLevel(){return this->pyramidLevel;};

			/**
			 * @fn	Initialize
			 * @brief
			 *		Initialize OpticalFlow
			 * @remark
	 		 *		it is necessary function 
			 *		initialize OpticalFlow setting pyramid level and window size
			 *		running opticalflow after initialization
			 */
			void Initialize(
							int width,							///< input image width size
							int height,							///< input image heightsize
							CvSize windowSize=cvSize(8, 8),		///< opticalflow window size
							int pyramidLevel=3					///< opticalflow pyramid level
							);

			/**
			 * @fn	TrackFeatures
			 * @brief
			 *		Tracking Feature using OpticalFlow
			 * @remark
			 *		Tracking Feature using OpticalFlow
			 * @return
			 *		success or failure
			 */
			int TrackFeatures(
							IplImage* prevGrayImage,					///< input image
							IplImage* currGrayImage,					///< input image
							std::vector<FeaturePoint>* prevPoints,		///< input previous points
							std::vector<FeaturePoint>* currentPoints	///< output updated points
							);
		};
		/** @} */ // addtogroup AlgorithmsOpticalFlow
		/** @} */ // addtogroup Algorithms
	}
}
#endif // _OPTICAL_FLOW_H_