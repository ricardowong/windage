/* ========================================================================
 * PROJECT: windage Features
 * ========================================================================
 * This work is based on the original windage Features developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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

#include "Algorithms/HarrisDetector.h"

using namespace windage::Algorithms;

bool HarrisDetector::DoExtractFeature(IplImage* grayImage)
{
	this->featurePoints.clear();

	IplImage* harrisResponse = cvCreateImage(cvGetSize(grayImage), IPL_DEPTH_32F, 1);
	cvCornerHarris(grayImage, harrisResponse, this->windowSize, 3, 0.1);

	double minValue, maxValue;
	cvMinMaxLoc(harrisResponse, &minValue, &maxValue);
	cvConvertScale(harrisResponse, harrisResponse, 255.0/(maxValue-minValue), minValue);
//	cvScale(harrisResponse, harrisResponse, 255.0/(maxValue-minValue), minValue);
	cvNamedWindow("test");
	cvShowImage("test", harrisResponse);

	for(int y=0; y<harrisResponse->height; y++)
	{
		for(int x=0; x<harrisResponse->width; x++)
		{
			float cornerness = CV_IMAGE_ELEM(harrisResponse, float, y, x);
			if(cornerness > 0)// this->threshold)
			{
				windage::FeaturePoint feature;
				feature.SetPoint(windage::Vector3(x, y, 0.0));
				feature.SetSize(5.0);
				this->featurePoints.push_back(feature);
			}
		}
	}

	cvReleaseImage(&harrisResponse);

	return true;
}