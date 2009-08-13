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