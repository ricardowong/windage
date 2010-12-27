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

#include "Algorithms/SURFDetector.h"

using namespace windage::Algorithms;

bool SURFDetector::DoExtractFeature(IplImage* grayImage)
{
	this->featurePoints.clear();

	CvSeq* keypoints;
	CvSeq* descriptors;
	CvMemStorage* storage = cvCreateMemStorage(0);

	CvSURFParams params = cvSURFParams(this->threshold, 1);
	cvExtractSURF(grayImage, NULL, &keypoints, &descriptors, storage, params);
	
	CvSeqReader reader;
	cvStartReadSeq(descriptors, &reader, 0);

	windage::FeaturePoint point;
	for(int i=0; i<keypoints->total; i++)
	{
		CvSURFPoint* surfPT = (CvSURFPoint*)cvGetSeqElem(keypoints, i);

		point.SetPoint(windage::Vector3(surfPT->pt.x, surfPT->pt.y, 1.0));
		point.SetSize(surfPT->size);
		point.SetDir(surfPT->dir);

		this->featurePoints.push_back(point);
	}

	cvReleaseMemStorage(&storage);

	return true;
}
