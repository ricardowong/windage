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

#ifndef _OPTICAL_FLOW_H_
#define _OPTICAL_FLOW_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>

namespace windage
{
	/**
	 * @brief
	 *		Class for OpticalFlow
	 * @author
	 *		windage
	 */
	class DLLEXPORT OpticalFlow
	{
	private:
		int imageWidth;		///< input image size
		int imageHeight;	///< input image size

		CvSize windowSize;	///< opticalflow window size
		int pyramidLevel;	///< opticalflow pyramid level
		
		const static int maxPointCount = 1000;	///< maximum feature points count
		CvPoint2D32f feature1[maxPointCount];	///< preview image feature points
		CvPoint2D32f feature2[maxPointCount];	///< current image feature points
		char foundFeature[maxPointCount];		///< tracking status
		float errorFeature[maxPointCount];

		CvTermCriteria terminationCriteria;
		IplImage* pyramid1;
		IplImage* pyramid2;

		bool removePrevPoints;

		void Release();
	public:
		OpticalFlow();
		~OpticalFlow();

		/**
		 * @brief
		 *		Set prev points remove mode
		 * @remark
		 *		default : false
		 */
		inline void SetRemovePrevPoints(bool removePrevPoints=true){this->removePrevPoints=removePrevPoints;};
		inline void SetImageSize(int width, int height){this->imageWidth=width;this->imageHeight=height;};
		inline CvSize GetImageSize(){return cvSize(this->imageWidth, this->imageHeight);};
		inline void SetWindowSize(CvSize size=cvSize(8, 8)){this->windowSize = size;};
		inline CvSize GetWindowSize(){return this->windowSize;};
		inline void SetPyramidLevel(int level=3){this->pyramidLevel = level;};
		inline int GetPyramidLevel(){return this->pyramidLevel;};

		/**
		 * @brief
		 *		Initialize OpticalFlow
		 * @remark
		 *		initialize OpticalFlow setting pyramid level and window size
		 *		running opticalflow after initialization
		 */
		void Initialize(
						int width,							///< input image width size
						int height,							///< input image heightsize
						CvSize windowSize=cvSize(10, 10),	///< opticalflow window size
						int pyramidLevel=3					///< opticalflow pyramid level
						);

		/**
		 * @brief
		 *		Tracking Feature using OpticalFlow
		 * @remark
		 *		Tracking Feature using OpticalFlow
		 */
		int TrackFeature(
						IplImage* prevGrayImage,						///< input image
						IplImage* currGrayImage,						///< input image
						std::vector<CvPoint2D32f>* prevPoints,		///< input previous points
						std::vector<CvPoint2D32f>* currentPoints	///< output updated points
						);
	};
}

#endif