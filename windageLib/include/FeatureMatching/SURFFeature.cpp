#include "SURFFeature.h"
#include "Tracker/FAST/wfastsurf.h"
using namespace windage;

SURFFeature::SURFFeature()
{
	scaleFactor = 3.0;
	scaleStep = 6;
	patch = NULL;
}

SURFFeature::~SURFFeature()
{
	this->Release();
}

void SURFFeature::Release()
{
	if(patch)
		cvReleaseImage(&patch);
	patch = NULL;
}

void SURFFeature::initialize(double scaleFactor, int scaleStep)
{
	this->Release();

	this->scaleFactor = scaleFactor;
	this->scaleStep = scaleStep;

	this->patch = cvCreateImage(cvSize(this->PATCH_WIDTH, this->PATCH_HEIGHT), IPL_DEPTH_8U, 1);
}

int SURFFeature::ExtractModifiedSURF(IplImage* grayImage, CvPoint center, Description* descriptions)
{
	CvSURFParams params = cvSURFParams(500, 0);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *referenceKeypoints = 0, *referenceDescriptors = 0;

	referenceKeypoints = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSURFPoint), storage );
	CvSURFPoint point = cvSURFPoint( cvPoint2D32f(center.x, center.y), 0, 15, 0, 0);
	cvSeqPush(referenceKeypoints, &point);

	wExtractFASTSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);
//	wExtractSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);

	CvSeqReader reader;
	cvStartReadSeq( referenceDescriptors, &reader, 0 );
	int length = (int)(referenceDescriptors->elem_size/sizeof(float));
	int count = referenceKeypoints->total;
	for(int i=0; i<count; i++ )
	{
		Description description;

		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( referenceKeypoints, i );

		float* vec = (float*)reader.ptr;
		for(int i=0; i<length; i++)
		{
			descriptions->descriptor[i] = vec[i];
		}
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
	}

	cvReleaseMemStorage(&storage);
	return count;
}

int SURFFeature::GenerateDescriptor(IplImage* grayImage, CvPoint point)
{
	int halfWidth = PATCH_WIDTH/2;
	int halfHeight = PATCH_HEIGHT/2;

	if( halfWidth <= point.x && point.x < grayImage->width - halfWidth &&
		halfHeight <= point.y && point.y < grayImage->height - halfHeight)
	{
		// save feature patch
		cvSetImageROI(grayImage, cvRect(point.x - halfWidth, point.y - halfHeight, PATCH_WIDTH, PATCH_HEIGHT));

		cvCopy(grayImage, this->patch);

		// generate descriptor
		int width = (int)((double)patch->width/scaleFactor);
		int height = (int)((double)patch->height/scaleFactor);

		// multi scale
		IplImage* tempReference;
		for(int y=1; y<=scaleStep; y++)
		{
			for(int x=1; x<=scaleStep; x++)
			{
				tempReference = cvCreateImage(cvSize(width*x, height*y), IPL_DEPTH_8U, 1);
				cvResize(this->patch, tempReference);

				Description description;
				if(this->ExtractModifiedSURF(tempReference, cvPoint((width*x)/2, (height*y)/2), &description) == 1)
					this->descriptionList.push_back(description);

				cvReleaseImage(&tempReference);
			}
		}

		cvResetImageROI(grayImage);
		return 1;
	}
	else
		return -1;
}

Description SURFFeature::GetDescription(int index)
{
	if(index < this->descriptionList.size())
		return descriptionList[index];
	else
		return Description();
}