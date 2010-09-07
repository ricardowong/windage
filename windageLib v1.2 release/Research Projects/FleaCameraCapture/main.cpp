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

#include <cv.h>
#include <highgui.h>

#include "../Common/FleaCamera.h"

void main()
{
	const int NUMBER_OF_CAMERA = 2;
	const int NUMBER_OF_IMAGE = 1000;
	const char* IMAGE_FILE_FORMAT = "capture/image_%d_%04d.png";
	char filename[1000];

	std::vector<FleaCamera*> capture;
	capture.resize(NUMBER_OF_CAMERA);
	for(int i=0; i<NUMBER_OF_CAMERA; i++)
	{
		capture[i] = new FleaCamera();
		capture[i]->open();
		capture[i]->start();

		sprintf_s(filename, IMAGE_FILE_FORMAT, i, 0);
		cvNamedWindow(filename);
	}

	bool preview = true;
	while(preview)
	{
		for(int i=0; i<NUMBER_OF_CAMERA; i++)
		{
			capture[i]->update();
			IplImage* inputImage = capture[i]->GetIPLImage();

			sprintf_s(filename, IMAGE_FILE_FORMAT, i, 0);
			cvShowImage(filename, inputImage);
		}

		switch(cvWaitKey(1))
		{
		case 's':
		case 'S':
		case ' ':
			preview = false;
		}
	}

	for(int j=0; j<NUMBER_OF_IMAGE; j++)
	{
		for(int i=0; i<NUMBER_OF_CAMERA; i++)
		{
			capture[i]->update();
			IplImage* inputImage = capture[i]->GetIPLImage();
			IplImage* colorImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 3);
			cvCvtColor(inputImage, colorImage, CV_BGRA2BGR);

			sprintf_s(filename, IMAGE_FILE_FORMAT, i, j);
			cvSaveImage(filename, colorImage);
			std::cout << "save image : " << filename << std::endl;

			sprintf_s(filename, IMAGE_FILE_FORMAT, i, 0);
			cvShowImage(filename, colorImage);

			cvReleaseImage(&colorImage);
		}

		switch(cvWaitKey(1))
		{
		case 'q':
		case 'Q':
			j = NUMBER_OF_IMAGE;
		}
	}

	for(int i=0; i<NUMBER_OF_CAMERA; i++)
	{
		capture[i]->stop();
		capture[i]->close();
		delete capture[i];
	}

	cvDestroyAllWindows();
}