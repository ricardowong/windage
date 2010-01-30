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

#include "FeatureMatching/FeatureFactory.h"
#include <omp.h>

#include "Tracker/FAST/fast.h"

using namespace windage;
int FeatureFactory::ExtractFASTCorner(IplImage* grayImage, int threshold, std::vector<CvPoint>* corners, int n)
{
	int cornerCount = 0;
	xy* cornertemp = NULL;

	bool isProcessed = true;
	switch(n)
	{
	case 9:
		cornertemp = fast9_detect((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 10:
		cornertemp = fast10_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 11:
		cornertemp = fast11_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 12 :
		cornertemp = fast12_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	default:
		isProcessed = false;
		break;
	}

	if(isProcessed)
	{
		for(int i=0; i<cornerCount; ++i)
		{
			corners->push_back(cvPoint(cornertemp[i].x, cornertemp[i].y));
		}

		if(cornertemp) delete cornertemp;
	}

	return (int)corners->size();
}

int FeatureFactory::Create2DPlaneSURFFeatureDescriptor(IplImage* grayImage, std::vector<CvPoint>* pointList, std::vector<SURFFeature*>* featureList, double scaleFactor, int scaleStep)
{
	for(int i=0; i<(int)(*pointList).size(); i++)
	{
		SURFFeature* feature = new SURFFeature();
		feature->initialize(scaleFactor, scaleStep);
		feature->SetPosition(Vector3((*pointList)[i].x, (*pointList)[i].y, 0.0));
		int result = feature->GenerateDescriptor(grayImage, (*pointList)[i]);

		if(result > 0)
			featureList->push_back(feature);
		else
			delete feature;
	}

	return featureList->size();
}