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

#include <vector>

#include <cv.h>
#include <highgui.h>

#include <windage.h>

const char* SAMPLE_FILE_NAME = "sample_1.png";

void main()
{
	windage::Logger logging(&std::cout);

	// datas
	IplImage* sampleImage = cvLoadImage(SAMPLE_FILE_NAME);
	IplImage* resultImage = cvCloneImage(sampleImage);
	IplImage* grayImage = cvCreateImage(cvGetSize(sampleImage), IPL_DEPTH_8U, 1);
	cvCvtColor(sampleImage, grayImage, CV_BGR2GRAY);

	cvNamedWindow("original");
	cvShowImage("original", sampleImage);

	// functions
	std::vector<windage::Algorithms::FeatureExtractor*> extractorList;
	extractorList.push_back(new windage::Algorithms::GoodFeatureToTrack());
	extractorList.push_back(new windage::Algorithms::HarrisDetector());
	extractorList.push_back(new windage::Algorithms::FASTDetector());
	extractorList.push_back(new windage::Algorithms::SURFDetector());
	extractorList.push_back(new windage::Algorithms::SIFTDetector());
	
	
	// do extract feature for each algorithms
	for(unsigned int i=0; i<extractorList.size(); i++)
	{
		IplImage* localResultImage = cvCloneImage(resultImage);

		logging.updateTickCount();
		extractorList[i]->DoExtractFeature(grayImage);
		double processingTime = logging.calculateProcessTime();

		std::vector<windage::FeaturePoint>* featureList = extractorList[i]->GetFeaturePoints();
		for(unsigned int j=0; j<featureList->size(); j++)
		{
			windage::Vector3 point = (*featureList)[j].GetPoint();
			double size = (*featureList)[j].GetSize();

			cvCircle(localResultImage, cvPoint(point.x, point.y), size, CV_RGB(0, 255, 0));
		}

		cvNamedWindow(extractorList[i]->GetFunctionName().c_str());
		cvShowImage(extractorList[i]->GetFunctionName().c_str(), localResultImage);

		char filename[100];
		sprintf(filename, "%s.jpg", extractorList[i]->GetFunctionName().c_str());
		cvSaveImage(filename, localResultImage);

		sprintf(filename, "%s : %0.2f ms", extractorList[i]->GetFunctionName().c_str(), processingTime);
		logging.log(filename);
		logging.logNewLine();

		cvReleaseImage(&localResultImage);
	}

	cvWaitKey();
	cvDestroyAllWindows();
}