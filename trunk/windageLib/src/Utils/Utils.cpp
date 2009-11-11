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

#include "Utils/Utils.h"
using namespace windage;

void Utils::DrawTextToImage(IplImage* colorImage, CvPoint position, char* message)
{
	CvFont outLineFont, font;
	cvInitFont(&outLineFont, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, 0.8, 0, 5);
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, 0.8, 0, 1);

	cvPutText(colorImage, message, position, &outLineFont, CV_RGB(0, 0, 0));
	cvPutText(colorImage, message, position, &font, CV_RGB(255, 255, 255));
}

bool IsSameColor(CvScalar color1, CvScalar color2)
{
	if( color1.val[0] == color2.val[0] &&
		color1.val[1] == color2.val[1] &&
		color1.val[2] == color2.val[2] &&
		color1.val[3] == color2.val[3])
		return true;
	else
		return false;
}

bool Utils::CompundImmersiveImage(IplImage* src, IplImage* dst, CvScalar maskColor, double alpha)
{
	if(src->width == dst->width && src->height == dst->height && src->depth == dst->depth)
	{
		for(int y=0; y<dst->height; y++)
		{
			for(int x=0; x<dst->width; x++)
			{
				CvScalar color = cvGet2D(src, y, x);
				if(!IsSameColor(color, maskColor))
				{
					CvScalar dstColor = cvGet2D(dst, y, x);
					for(int i=0; i<4; i++)
						color.val[i] = color.val[i] * alpha + dstColor.val[i] * (1.0 - alpha);
					cvSet2D(dst, y, x, color);
				}
			}
		}

		return true;
	}
	else
		return false;
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
//*
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]+size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]+size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]-size), CV_RGB(255, 0, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]+size, worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0], worldPoint.val[1]-size, worldPoint.val[2]-size), CV_RGB(255, 0, 0));
//*/
//*
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]+size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]+size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]+size),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]-size), CV_RGB(0, 255, 0));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1], worldPoint.val[2]-size),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1], worldPoint.val[2]-size), CV_RGB(0, 255, 0));
//*/
//*
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]-size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]+size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]+size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]+size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]+size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]-size, worldPoint.val[2]), CV_RGB(0, 0, 255));
	cvLine(colorImage,	calibration->ConvertWorld2Image(worldPoint.val[0]+size, worldPoint.val[1]-size, worldPoint.val[2]),
						calibration->ConvertWorld2Image(worldPoint.val[0]-size, worldPoint.val[1]-size, worldPoint.val[2]), CV_RGB(0, 0, 255));
//*/
}

IplImage* Utils::GeneratePatchMap(std::vector<SURFFeature*>* featureList)
{
	int count = featureList->size();
	if(count > 0)
	{
		int widthCount = sqrt((double)count) + 1;
		int width = widthCount * (*featureList)[0]->GetPatchWidth();

		IplImage* temp = cvCreateImage(cvSize(width, width), IPL_DEPTH_8U, 1);
		cvZero(temp);

		for(int i=0; i<featureList->size(); i++)
		{
			int x = i%widthCount;
			int y = i/widthCount;

			cvSetImageROI(temp, cvRect(x*(*featureList)[i]->GetPatchWidth(), y*(*featureList)[i]->GetPatchHeight(), (*featureList)[i]->GetPatchWidth(), (*featureList)[i]->GetPatchHeight()));
			cvCopy((*featureList)[i]->GetPatch(), temp);
		}
		cvResetImageROI(temp);

		return temp;
	}
	else
	{
		return NULL;
	}
}

bool Utils::IsInside(CvPoint point, CvPoint corner1, CvPoint corner2, CvPoint corner3, CvPoint corner4, bool isClockWise)
{
	int line1 = (corner1.x*corner2.y + corner2.x*point.y + point.x*corner1.y)
			  - (corner1.y*corner2.x + corner2.y*point.x + point.y*corner1.x);

	int line2 = (corner2.x*corner3.y + corner3.x*point.y + point.x*corner2.y)
			  - (corner2.y*corner3.x + corner3.y*point.x + point.y*corner2.x);

	int line3 = (corner3.x*corner4.y + corner4.x*point.y + point.x*corner3.y)
			  - (corner3.y*corner4.x + corner4.y*point.x + point.y*corner3.x);

	int line4 = (corner4.x*corner1.y + corner1.x*point.y + point.x*corner4.y)
			  - (corner4.y*corner1.x + corner1.y*point.x + point.y*corner4.x);
	
	if(isClockWise)
	{
		if(line1 < 0 && line2 < 0 && line3 < 0 && line4 < 0)
			return true;
		else
			return false;
	}
	else
	{
		if(line1 > 0 && line2 > 0 && line3 > 0 && line4 > 0)
			return true;
		else
			return false;
	}
}