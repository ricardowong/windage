/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
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

#include <iostream>
#include <vector>

#include <gl/glut.h>
#include <cv.h>
#include <highgui.h>

#include <windage.h>

void main()
{
	const int NUMBER_OF_POINTS = 100;
	const int ERROR_POINTs = NUMBER_OF_POINTS/10;

	windage::Logger logger(&std::cout);
	std::vector<windage::Algorithms::HomographyEstimator*> estimators;

	estimators.push_back(new windage::Algorithms::RANSACestimator());
	estimators.push_back(new windage::Algorithms::ProSACestimator());
	estimators.push_back(new windage::Algorithms::LMedSestimator());

	std::vector<windage::FeaturePoint> refPoints;
	std::vector<windage::FeaturePoint> scePoints;

	CvRNG rng = cvRNG(cvGetTickCount());
	windage::Matrix3 homography;
	for(int i=0; i<9; i++)
	{
		homography.m1[i] = (float)(cvRandInt(&rng)%100)/10.0f;
	}
	homography.m1[8] = 1.0f;
//	homography._11 = homography._22 = homography._33 = 1.0f;

	// generate matching sets
	for(int i=0; i<NUMBER_OF_POINTS; i++)
	{	
		float x1 = (float)(cvRandInt(&rng)%100)/10.0f;
		float y1 = (float)(cvRandInt(&rng)%100)/10.0f;
		windage::Vector3 pt1(x1, y1, 1.0f);
		windage::Vector3 pt2 = homography * pt1;
		pt2.x /= pt1.z;
		pt2.y /= pt1.z;

		float dx = (float)(cvRandInt(&rng)%10)/10.0f;
		float dy = (float)(cvRandInt(&rng)%10)/10.0f;
		pt2.x += dx;
		pt2.y += dy;

		windage::FeaturePoint refPt, scePt;
		refPt.SetPoint(pt1);
		refPt.SetDistance(10.0f);
		scePt.SetPoint(pt2);
		scePt.SetDistance(10.0f);

		refPoints.push_back(refPt);
		scePoints.push_back(scePt);
	}

	for(int i=0; i<8; i++)
	{
		for(int j=0; j<i*ERROR_POINTs; j++)
		{
			float x1 = (float)(cvRandInt(&rng)%1000)/10.0f;
			float y1 = (float)(cvRandInt(&rng)%1000)/10.0f;

			scePoints[j].SetPoint(windage::Vector3(x1, y1, 1.0f));
			refPoints[j].SetDistance(100.0f);
			scePoints[j].SetDistance(100.0f);
		}

		logger.log("error(%)", i*10);
		logger.logNewLine();

		for(int j=0; j<estimators.size(); j++)
		{
			logger.log(estimators[j]->GetFunctionName());
			logger.logNewLine();

			estimators[j]->AttatchReferencePoint(&refPoints);
			estimators[j]->AttatchScenePoint(&scePoints);

			logger.updateTickCount();
			estimators[j]->Calculate();
			double processingTime = logger.calculateProcessTime();

			windage::Matrix3 eHomography = homography - (*estimators[j]->GetHomography());

			float error = 0.0f;
			for(int k=0; k<9; k++)
			{
				error += fabs(eHomography.m1[k]);
			}

			logger.log("processingTime", processingTime);
			logger.log("error", error);
			logger.logNewLine();
		}
		logger.logNewLine();
	}
}