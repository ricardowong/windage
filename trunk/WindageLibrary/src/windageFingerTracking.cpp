#include <cv.h>
#include <vector>
#include "../include/windageVector.h"

#define M_PI 3.14159265

extern "C" __declspec(dllexport)
void SkinDetection(IplImage* resultImage, IplImage* inputImage, int cbThresholdU, int cbThresholdL, int crThresholdU, int crThresholdL)
{
	IplImage* ycrcbImage = cvCreateImage(cvSize(inputImage->width, inputImage->height), inputImage->depth, inputImage->nChannels);
	cvCvtColor(inputImage, ycrcbImage, CV_BGR2YCrCb);

	int x, y;
	unsigned char cr, cb;
	for(y=0; y<resultImage->height; y++)
	{
		for(x=0; x<resultImage->width; x++)
		{
			cr = ycrcbImage->imageData[y*ycrcbImage->widthStep+3*x+1];
			cb = ycrcbImage->imageData[y*ycrcbImage->widthStep+3*x+2];

			if(cbThresholdL <= cb && cb <= cbThresholdU && crThresholdL <= cr && cr <= crThresholdU)
			{
				resultImage->imageData[y*resultImage->widthStep+3*x+2] = (unsigned char)255;
				resultImage->imageData[y*resultImage->widthStep+3*x+1] = (unsigned char)255;
				resultImage->imageData[y*resultImage->widthStep+3*x+0] = (unsigned char)255;
			}
			else
			{
				resultImage->imageData[y*resultImage->widthStep+3*x+2] = (unsigned char)0;
				resultImage->imageData[y*resultImage->widthStep+3*x+1] = (unsigned char)0;
				resultImage->imageData[y*resultImage->widthStep+3*x+0] = (unsigned char)0;
			}
		}
	}

	cvReleaseImage(&ycrcbImage);
}

extern "C" __declspec(dllexport)
double HandDetection(IplImage* result, Vector3* centerPosition,  IplImage* input)
{
	IplImage* grayImage = cvCreateImage(cvSize(input->width, input->height), IPL_DEPTH_8U, 1);
	cvCvtColor(input, grayImage, CV_BGR2GRAY);
	IplImage* binaryImage = cvCreateImage(cvSize(input->width, input->height), IPL_DEPTH_8U, 1);
	cvThreshold(grayImage, binaryImage, 100, 255, CV_THRESH_BINARY);

	CvMemStorage* stor = cvCreateMemStorage(0);
	CvSeq* contour = 0;//cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);
	cvFindContours(binaryImage, stor, &contour);

	CvSeq* maxIterator = NULL;
	double maxSize = 0;
	double area;
	for(; contour!=0; contour=contour->h_next)
	{
		area = fabs(cvContourArea(contour, CV_WHOLE_SEQ));
		if(maxSize < area)
		{
			maxSize = area;
			maxIterator = contour;
		}
	}

	cvZero(result);
	if(maxIterator != NULL)
	{
		if(fabs(cvContourArea(maxIterator, CV_WHOLE_SEQ)) > 100)
		{
			cvDrawContours(result, maxIterator, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), -1, 1, 8);

			// find center of weight
			IplImage* temp = cvCreateImage(cvSize(input->width, input->height), input->depth, input->nChannels);
			cvZero(temp);
			cvDrawContours(temp, maxIterator, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), -1, CV_FILLED, 8);

			int xPoints=0;
			int yPoints=0;
			int count = 0;
			int x, y;

			for(y=0; y<temp->height; y++)
			{
				for(x=0; x<temp->width; x++)
				{
					if(temp->imageData[y*temp->widthStep+3*x+2] == (char)255)
					{
						xPoints += x;
						yPoints += y;
						count++;
					}
				}
			}

			centerPosition->x = ((double)xPoints/(double)count);
			centerPosition->y = ((double)yPoints/(double)count);
			centerPosition->z = (double)count;

			cvReleaseImage(&temp);
		}
	}

	cvReleaseImage(&grayImage);
	cvReleaseImage(&binaryImage);
	cvReleaseMemStorage(&stor);

	return centerPosition->z;
}

extern "C" __declspec(dllexport)
void ConvexHull(std::vector<Vector3>* pointList, IplImage* input, double distanceThreshold, IplImage* debugImage)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* ptseq = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour), sizeof(CvPoint), storage );
	CvSeq* hull;

	CvPoint pt, pt0, pt1;
	int x, y;
	for(y=0; y<input->height; y++)
	{
		for(x=0; x<input->width; x++)
		{
			if(input->imageData[y*input->widthStep+x*3+2] == (char)255)
			{
				pt0.x = x;
				pt0.y = y;
				cvSeqPush( ptseq, &pt0 );
			}
		}
	}
    
	hull = cvConvexHull2( ptseq, 0, CV_CLOCKWISE, 0 );
	if(hull == NULL)
		return;

	int hullcount = hull->total;

	int i, j;
	int startCount;
	CvPoint npt;
	double distance1, distance2;
	bool beforeConnect, afterConnect;

	Vector3 point;

	pt0 = **CV_GET_SEQ_ELEM( CvPoint*, hull, hullcount - 1 );
	pt = **CV_GET_SEQ_ELEM( CvPoint*, hull, 0 );

	distance2 = sqrt( pow((double)(pt.x-pt0.x), 2) + pow((double)(pt.y-pt0.y), 2) );
	if(distance2 < distanceThreshold)
		afterConnect = true;
	else
		afterConnect = false;

	startCount = 0;
	for(i=0; i<=hullcount; i++ )
    {                
		j = (i+1)%hullcount;
        pt = **CV_GET_SEQ_ELEM( CvPoint*, hull, i );
		pt1 = **CV_GET_SEQ_ELEM( CvPoint*, hull, j );

		distance1 = distance2;
		distance2 = sqrt( pow((double)(pt.x-pt1.x), 2) + pow((double)(pt.y-pt1.y), 2) );

		beforeConnect = afterConnect;
		if(distance2 < distanceThreshold)
			afterConnect = true;
		else
			afterConnect = false;

		if(beforeConnect == false && afterConnect == false)
		{
			if(debugImage != NULL)
			{
				cvCircle(debugImage, pt, 5, CV_RGB(0, 255/hullcount*(hullcount - i), 255/hullcount*i));
			}

			point.x = pt.x;
			point.y = pt.y;
			point.z = 1;
			pointList->push_back(point);
		}
		else if(beforeConnect == false && afterConnect == true)
		{
			startCount = i;
		}
		else if(beforeConnect == true && afterConnect == false)
		{
			npt = **CV_GET_SEQ_ELEM( CvPoint*, hull, (int)(startCount + abs(i - startCount)/2));

			if(debugImage != NULL)
			{
				cvCircle(debugImage, npt, 5, CV_RGB(0, 255/hullcount*(hullcount - i), 255/hullcount*i));
			}

			point.x = npt.x;
			point.y = npt.y;
			point.z = 1;
			pointList->push_back(point);
		}
		else // beforeConnect == true && afterConnect == true
		{
		}

		if(debugImage != NULL)
		{
			cvLine(debugImage, pt0, pt, CV_RGB(0, 255/hullcount*(hullcount - i), 255/hullcount*i));
		}
		
		pt0 = pt;
	}

	cvReleaseMemStorage(&storage);
}

extern "C" __declspec(dllexport)
void FindFingerTipPoint(std::vector<Vector3>* result, std::vector<Vector3>* pointList, Vector3 centerPoint, double radius, int border, IplImage* inputImage, IplImage* debugImage)
{
	int width = 320, height = 240;

	std::vector<Vector3> firstResult;
	firstResult.clear();

	Vector3 point;
	int count = (int)pointList->size();
	for(int i=0; i<count; i++)
	{
		point = (*pointList)[i];

		if(border < point.x && point.x < width-border && border < point.y && point.y < height-border)
		{
			if(radius < sqrt( pow(centerPoint.x - point.x, 2) + pow(centerPoint.y - point.y, 2) ) )
			{
				firstResult.push_back(point);
			}
		}
	}

	double maxDistance = 0;
	int maxIndex=0;
	count = (int)firstResult.size();

	for(int i=0; i<count; i++)
	{
		point = firstResult[i];
		std::vector<Vector2> outline;
		Vector2 out;

		int x, y, xdot, ydot;
		int windowSize = (int)(radius*0.25f);
		double degreeThreshold = 100.0f;

		x = -windowSize;
		for(y=-windowSize; y<windowSize; y++)
		{
			xdot = x + (int)point.x;
			ydot = y + (int)point.y;

			if(0 < ydot && ydot < inputImage->height && 0 < xdot && xdot < inputImage->width)
			{
				if(inputImage->imageData[ydot * inputImage->widthStep + xdot * 3 + 2] == (char)255)
				{
					out.x = (double)xdot;
					out.y = (double)ydot;

					outline.push_back(out);
				}
			}
		}

		x = windowSize;
		for(y=-windowSize+1; y<windowSize+1; y++)
		{
			xdot = x + (int)point.x;
			ydot = y + (int)point.y;

			if(0 < ydot && ydot < inputImage->height && 0 < xdot && xdot < inputImage->width)
			{
				if(inputImage->imageData[ydot * inputImage->widthStep + xdot * 3 + 2] == (char)255)
				{
					out.x = (double)xdot;
					out.y = (double)ydot;

					outline.push_back(out);
				}
			}
		}

		y = -windowSize;
		for(x=-windowSize+1; x<windowSize+1; x++)
		{
			xdot = x + (int)point.x;
			ydot = y + (int)point.y;

			if(0 < ydot && ydot < inputImage->height && 0 < xdot && xdot < inputImage->width)
			{
				if(inputImage->imageData[ydot * inputImage->widthStep + xdot * 3 + 2] == (char)255)
				{
					out.x = (double)xdot;
					out.y = (double)ydot;

					outline.push_back(out);
				}
			}
		}

		y = windowSize;
		for(x=-windowSize; x<windowSize; x++)
		{
			xdot = x + (int)point.x;
			ydot = y + (int)point.y;

			if(0 < ydot && ydot < inputImage->height && 0 < xdot && xdot < inputImage->width)
			{
				if(inputImage->imageData[ydot * inputImage->widthStep + xdot * 3 + 2] == (char)255)
				{
					out.x = (double)xdot;
					out.y = (double)ydot;

					outline.push_back(out);
				}
			}
		}

		int ocount = (int)outline.size();
		if(ocount >= 2)
		{
			Vector2 point1 = outline[0];
			Vector2 point2 = outline[ocount-1];

			if(debugImage != NULL)
			{
				cvCircle(debugImage, cvPoint((int)point1.x, (int)point1.y), 2, CV_RGB(255, 0, 0), CV_FILLED);
				cvCircle(debugImage, cvPoint((int)point2.x, (int)point2.y), 2, CV_RGB(255, 0, 0), CV_FILLED);
			}

			point1.x = point1.x - point.x;
			point1.y = point1.y - point.y;
			point2.x = point2.x - point.x;
			point2.y = point2.y - point.y;

			double a = point1 * point2;
			double b = sqrt( pow(point1.x, 2) + pow(point1.y, 2) ) * sqrt( pow(point2.x, 2) + pow(point2.y, 2) );
			double input =  a / b;
			double degree = (180.0f / M_PI) * acos( input );
			
			if(degree < degreeThreshold)
			{
				point.z = 1;
				result->push_back(point);
			}
		}
		else
		{
			point.z = 1;
			result->push_back(point);
		}
	}

}