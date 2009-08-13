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
		int GenerateReferenceFeatureTree();

		int ExtractFASTCorner(std::vector<CvPoint>* corners, IplImage* grayImage, int threshold, int n=12);
		int ExtractModifiedSURF(IplImage* grayImage, std::vector<SURFDesciription>* descriptions, int thresholdFAST);
		CvFeatureTree* CreateReferenceTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage);
		int FindPairs(SURFDesciription description, CvFeatureTree* tree);
		int FindPairs(SURFDesciription description, std::vector<SURFDesciription>* descriptions);

		double CalculatePose();

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

		void Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, IplImage* referenceImage, double realWidth, double realHeight, int featureExtractThreshold=100);

		int UpdateCameraPose(IplImage* grayImage);
		void DrawDebugInfo(IplImage* colorImage);

		// optical flow
		void InitializeOpticalFlow(int width, int height, int stepSize=10, CvSize windowSize=cvSize(10, 10), int pyramidLevel=3);
		inline void SetOpticalFlowRunning(bool run){this->runOpticalflow = run;};
	};
}

#endif