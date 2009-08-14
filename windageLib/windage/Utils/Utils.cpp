#include <stdio.h>

#include "Utils.h"
using namespace windage;

void Utils::DrawTextToImage(IplImage* colorImage, CvPoint position, char* message)
{
	CvFont outLineFont, font;
	cvInitFont(&outLineFont, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, 0.8, 0, 5);
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, 0.8, 0, 1);

	cvPutText(colorImage, message, position, &outLineFont, CV_RGB(0, 0, 0));
	cvPutText(colorImage, message, position, &font, CV_RGB(255, 255, 255));
}

void Utils::DrawWorldCoordinatePoint(IplImage* colorImage, Calibration* calibration, CvScalar worldPoint, double size, bool drawText)
{
	if(drawText)
	{
		CvPoint projection = calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]+size, worldPoint.val[2]+size);
		char message[100];
		sprintf(message, "%d, %d, %d", (int)worldPoint.val[0], (int)worldPoint.val[1], (int)worldPoint.val[2]);
		Utils::DrawTextToImage(colorImage, projection, message);
	}

	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]+size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]+size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]-size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]-size), CV_RGB(255, 0, 0));

	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]+size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]+size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]-size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]-size), CV_RGB(0, 255, 0));

	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]-size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]+size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]+size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]+size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]+size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]-size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]-size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]-size, worldPoint.val[2]), CV_RGB(0, 0, 255));

}