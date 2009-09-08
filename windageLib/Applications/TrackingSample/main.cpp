//#define RUNNING
#ifdef RUNNING

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

#include <iostream>

#include <highgui.h>
#include <windage.h>

#define FLIP
#define RECTIFICATION

const int WIDTH = 640;
const int HEIGHT = 480;

void main()
{
	windage::Logger* log = new windage::Logger(&std::cout);
	windage::Logger* fpslog = new windage::Logger(&std::cout);

	// Tracker Initialize
	IplImage* referenceImage = cvLoadImage("reference.png", 0);

	windage::Tracker* tracker = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 30);
	((windage::ModifiedSURFTracker*)tracker)->RegistReferenceImage(referenceImage, 26.70, 20.00, 4.0, 6);
	((windage::ModifiedSURFTracker*)tracker)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker)->SetOpticalFlowRunning(true);
	((windage::ModifiedSURFTracker*)tracker)->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	cvNamedWindow("result");
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);

	char message[100];

	fpslog->updateTickCount();
	bool processing = true;
	while(processing)
	{
		double fps = fpslog->calculateFPS();
		fpslog->updateTickCount();
		
		// camera frame grabbing and convert to gray color
		log->updateTickCount();
		IplImage* grabFrame = cvQueryFrame(capture);
#ifdef FLIP
		cvFlip(grabFrame, grabFrame);
#endif
#ifdef RECTIFICATION
		tracker->GetCameraParameter()->Undistortion(grabFrame, inputImage);
#else
		cvCopy(grabFrame, inputImage);
#endif

		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();
		int result = tracker->UpdateCameraPose(grayImage);

		// draw tracking result
//		tracker->DrawDebugInfo(inputImage);
		tracker->DrawInfomation(inputImage, 10.0);

		log->log("tracking", log->calculateProcessTime());
		log->log("featureCount", result);
		log->logNewLine();

		sprintf(message, "FPS : %lf, Feature Count : %d", fps, result);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(10, 20), message);

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'p':
		case 'P':
			{
				std::vector<CvPoint> points;
				windage::ModifiedSURFTracker::ExtractFASTCorner(&points, grayImage, 45);
				for(int i=0; i<points.size(); i++)
					cvCircle(inputImage, points[i], 3, CV_RGB(255, 0, 0));
			}
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
	}

	cvReleaseCapture(&capture);
}

#endif