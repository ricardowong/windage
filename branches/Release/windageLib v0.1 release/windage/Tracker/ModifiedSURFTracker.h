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

#ifndef _MODIFIED_SURF_TRACKER_H_
#define _MODIFIED_SURF_TRACKER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>
#include "Tracker.h"
#include "OpticalFlow.h"

namespace windage
{
	const int DESCRIPTOR_DIEMNSION = 36;
	typedef struct _SURFDescription
	{
		CvPoint2D32f point;
		int size;
		double descriptor[DESCRIPTOR_DIEMNSION];

		void operator=(struct _SURFDescription oprd)
		{
			point = oprd.point;
			size = oprd.size;
			for(int i=0; i<DESCRIPTOR_DIEMNSION; i++)
				descriptor[i] = oprd.descriptor[i];
		}
		double distance(struct _SURFDescription oprd)
		{
			double sum = 0;
			for(int i=0; i<DESCRIPTOR_DIEMNSION; i++)
				sum += (double)((descriptor[i] - oprd.descriptor[i]) * (descriptor[i] - oprd.descriptor[i]));
			return sum;
		}
	}SURFDesciription;

	class DLLEXPORT ModifiedSURFTracker : public Tracker
	{
	private:
		double realWidth;
		double realHeight;
		int featureExtractThreshold;

		IplImage* referenceImage;
		std::vector<SURFDesciription> referenceSURF;
		std::vector<SURFDesciription> sceneSURF;

		std::vector<SURFDesciription> matchedReference;
		std::vector<SURFDesciription> matchedScene;
		std::vector<CvPoint2D32f> matchedReferencePoints;
		std::vector<CvPoint2D32f> matchedScenePoints;

		void Release();

		CvMat* referenceFeatureStorage;
		CvFeatureTree* referenceFeatureTree;
		int GenerateReferenceFeatureTree(double scaleFactor=4.0, int scaleStep=8);

		CvFeatureTree* CreateReferenceTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage);
		int FindPairs(SURFDesciription description, CvFeatureTree* tree, double distanceRate=0.7);
		int FindPairs(SURFDesciription description, std::vector<SURFDesciription>* descriptions);

		double CalculatePose(bool update=true);

		// optical flow
		bool runOpticalflow;
		unsigned int stepSize;
		unsigned int step;
		OpticalFlow* opticalflow;

	public:
		ModifiedSURFTracker();
		virtual ~ModifiedSURFTracker();

		inline IplImage* GetReferenceImage(){return this->referenceImage;};
		inline int GetFeatureExtractTreshold(){return this->featureExtractThreshold;};
		inline void SetFeatureExtractTreshold(int threshold=80){this->featureExtractThreshold = threshold;};

		void Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, int featureExtractThreshold=100);
		void RegistReferenceImage(IplImage* referenceImage, double realWidth, double realHeight, double scaleFactor=4.0, int scaleStep=8);

		static int ExtractFASTCorner(std::vector<CvPoint>* corners, IplImage* grayImage, int threshold, int n=9);
		static int ExtractModifiedSURF(IplImage* grayImage, std::vector<CvPoint>* corners, std::vector<SURFDesciription>* descriptions);

		int UpdateCameraPose(IplImage* grayImage);
		void DrawDebugInfo(IplImage* colorImage);

		// optical flow
		void InitializeOpticalFlow(int width, int height, int stepSize=10, CvSize windowSize=cvSize(10, 10), int pyramidLevel=3);
		inline void SetOpticalFlowRunning(bool run){this->runOpticalflow = run;};
	};
}

#endif