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