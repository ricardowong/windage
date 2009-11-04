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

#ifndef _MULTIPLE_SURF_TRACKER_H_
#define _MULTIPLE_SURF_TRACKER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>
#include <cvaux.h>

#include "Tracker.h"
#include "MultipleTracker.h"
#include "OpticalFlow.h"
#include "ModifiedSURFTracker.h"

#include "ReferenceStorage.h"

namespace windage
{
	class DLLEXPORT MultipleSURFTracker : public MultipleTracker
	{
	private:
		int featureExtractThreshold;	///< FAST Corner extract threshold value
		int featureCount;
		POSE_ESTIMATION_METHOD poseEstimationMethod;
		bool outlinerRemove;

		int step;
		IplImage* prevImage;
		int referenceCount;

		std::vector<SURFReferenceStorage*> referenceStorageList;
		std::vector<std::vector<SURFDesciription>> sceneSURF;		///< input image surf description
		std::vector<std::vector<int>> matchedReferenceIndex;		///< optical flow matched reference point
//		std::vector<int> matchedCount;

		void Release();
		
	public:
		MultipleSURFTracker()
		{
			featureExtractThreshold = 30;
			poseEstimationMethod = windage::RANSAC;
			outlinerRemove = true;

			step = 0;
			prevImage = NULL;
			referenceCount = 0;
		}
		~MultipleSURFTracker()
		{
			this->Release();
		}

		inline int GetTrackerCount(){return referenceCount;};
		inline void SetFeatureExtractThreshold(int threshold){this->featureExtractThreshold = threshold;};
		inline int GetFeatureCount(){return this->featureCount;};
		inline int GetMatchedCount(int index){return this->matchedReferenceIndex[index].size();};
		inline void SetPoseEstimationMethod(POSE_ESTIMATION_METHOD poseEstimationMethod=windage::RANSAC){this->poseEstimationMethod = poseEstimationMethod;};
		inline POSE_ESTIMATION_METHOD GetPoseEstimationMethod(){return this->poseEstimationMethod;};
		inline void SetOutlinerRemove(bool remove){this->outlinerRemove = remove;};

		void AttatchReferenceImage(IplImage* image, double realWidth, double realHeight, double scaleFactor=4.0, int scaleStep=8);
		bool DeleteReferenceImage(int index);

		int DetectObject(std::vector<SURFDesciription>* scene, std::vector<int>* matchedIndex, int index);
		double CalculatePose(int index);
		double UpdateCameraPose(IplImage* grayImage);
		void DrawDebugInfo(IplImage* colorImage);
		void DrawDebugInfo2(IplImage* colorImage, int index);
		void DrawOutLine(IplImage* colorImage, int index, bool drawCross = false);
		void DrawInfomation(IplImage* colorImage, int index, double size = 10.0);
	};
}

#endif