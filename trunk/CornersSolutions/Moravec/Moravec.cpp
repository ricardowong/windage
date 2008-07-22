// Moravec.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#define WINDOW_SIZE 3

int _tmain(int argc, _TCHAR* argv[])
{
	IplImage* inputImage = cvLoadImage("../data/test.pgm");
	IplImage* grayImage = cvCreateImage(cvSize(inputImage->width, inputImage->height), IPL_DEPTH_8U, 1);
	cvCvtColor(inputImage, grayImage, CV_BGR2GRAY);

	IplImage* cornernessMap = cvCreateImage(cvSize(inputImage->width, inputImage->height), IPL_DEPTH_32F, 1);
	IplImage* resultImage = cvCloneImage(inputImage);

	int x, y, shiftX, shiftY;
	int u, v;
	int width, height, window;
	width = inputImage->width;
	height = inputImage->height;
	window = (int)(WINDOW_SIZE/2);

	double baseWindow[WINDOW_SIZE][WINDOW_SIZE];
	double resultWindow[WINDOW_SIZE][WINDOW_SIZE];

	CvScalar temp;
	double sum;
	for(y=(window*2); y<height-(window*2); y++)
	{
		for(x=(window*2); x<width-(window*2); x++)
		{
			// save base window
			for(v=-window; v<=window; v++)
			{
				for(u=-window; u<=window; u++)
				{
					temp = cvGet2D(grayImage, x+u, y+v);
					baseWindow[window+u][window+v] = temp.val[0]/255.0f;
				}
			}

			for(shiftY=-1; shiftY<=1; shiftY++)
			{
				for(shiftX=-1; shiftX<=1; shiftX++)
				{
					sum = 0;

					if(shiftX != 0 || shiftY != 0)
					{
						for(v=-window; v<=window; v++)
						{
							for(u=-window; u<=window; u++)
							{
								temp = cvGet2D(grayImage, x+shiftX+u, y+shiftY+v);
								sum += pow(temp.val[0]/255.0f - baseWindow[window+u][window+v], 2);
							}
						}

						resultWindow[1+shiftX][1+shiftY] = sum;
					} // notIf(0, 0)
				}
			}

			// find min value;
			sum = 100.0f;
			for(v=-1; v<=1; v++)
			{
				for(u=-1; u<=1; u++)
				{
					if((u !=0 || v != 0) &&
						sum > resultWindow[1+u][1+v])
					{
						sum = resultWindow[1+u][1+v];
					}
				}
			}

			if(0.003f < sum && sum < 1.5f)
			{
				sum = 1.0f;
				cvRectangle(resultImage, cvPoint(y-2, x-2), cvPoint(y+2, x+2), CV_RGB(255, 0, 0));
			}
			else
			{
				sum = 0.0f;
			}
			cvSet2D(cornernessMap, x, y, cvScalar(sum));
		}
	}

	// Draw Data
	cvNamedWindow("Moravec Test Window");
	cvResizeWindow("Moravec Test Window", inputImage->width, inputImage->height);

//	cvShowImage("Moravec Test Window", grayImage);
//	cvShowImage("Moravec Test Window", cornernessMap);
	cvShowImage("Moravec Test Window", resultImage);
	
	cvWaitKey();

	cvDestroyAllWindows();

	return 0;
}

