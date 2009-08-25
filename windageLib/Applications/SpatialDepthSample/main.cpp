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
#include <vector>

#include <omp.h>

#include "PGRCamera.h"
#include "Tracker/ModifiedSURFTracker.h"
#include "SpatialInteraction/StereoSpatialSensor.h"
#include "SpatialInteraction/StereoSURFSpatialSensor.h"

#define SPATIAL_SENSOR_TYPE StereoSURFSpatialSensor
#define UNDISTORTION

const int WIDTH = 640;
const int HEIGHT = 480;

const int SPATIAL_X = 120;
const int SPATIAL_Y = 75;
const int SPATIAL_Z = 50;
const int SPACING = 2;
const double ACTIVATION_TRESHOLD = 0.2;

void main()
{
	// connect camera
	CPGRCamera* camera1 = new CPGRCamera();
	CPGRCamera* camera2 = new CPGRCamera();
	camera1->open();
	camera2->open();
	camera1->start();
	camera2->start();

	IplImage* temp1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* input1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* temp2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* input2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* gray2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	cvNamedWindow("image1");
	cvNamedWindow("image2");
	cvNamedWindow("spatial");

	// initialize tracker
	IplImage* referenceImage = cvLoadImage("reference.png", 0);
	windage::Tracker* tracker1 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker1)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker1)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker1)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker1)->SetOpticalFlowRunning(true);
	
	windage::Tracker* tracker2 = new windage::ModifiedSURFTracker();
	((windage::ModifiedSURFTracker*)tracker2)->Initialize(778.195, 779.430, 324.659, 235.685, -0.333103, 0.173760, 0.000653, 0.001114, 45);
	((windage::ModifiedSURFTracker*)tracker2)->RegistReferenceImage(referenceImage, 267.0, 200.0, 4.0, 8);
	((windage::ModifiedSURFTracker*)tracker2)->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(15, 15), 3);
	((windage::ModifiedSURFTracker*)tracker2)->SetOpticalFlowRunning(true);

	// for spatial sensor
	std::vector<windage::SpatialSensor*> spatialSensors;
	IplImage* spatialImage = cvCreateImage(cvSize(SPATIAL_X, SPATIAL_Y), IPL_DEPTH_8U, 1);
	cvZero(spatialImage);
	for(int z=0; z<SPATIAL_Z; z++)
	{
		for(int y=20; y<SPATIAL_Y+20; y++)
		{
			for(int x=0; x<SPATIAL_X; x++)
			{
				windage::SPATIAL_SENSOR_TYPE* tempSpatialSensor = new windage::SPATIAL_SENSOR_TYPE();
				tempSpatialSensor->Initialize(windage::Vector3(x*SPACING, y*SPACING, z*SPACING), ACTIVATION_TRESHOLD);
				tempSpatialSensor->AttatchCameraParameter(0, tracker1->GetCameraParameter());
				tempSpatialSensor->AttatchCameraParameter(1, tracker2->GetCameraParameter());

				spatialSensors.push_back(tempSpatialSensor);
			}
		}
	}

	bool processing = true;
	bool isSpatialProcessing = false;
	while(processing)
	{
		// camera frame grabbing
		camera1->update();
		camera2->update();

#ifdef UNDISTORTION
		tracker1->GetCameraParameter()->Undistortion(camera1->GetIPLImage(), temp1);
		tracker2->GetCameraParameter()->Undistortion(camera2->GetIPLImage(), temp2);
#else
		cvCopy(camera1->GetIPLImage(), temp1);
		cvCopy(camera2->GetIPLImage(), temp2);
#endif
		cvResize(temp1, input1);
		cvResize(temp2, input2);
		cvCvtColor(input1, gray1, CV_BGRA2GRAY);
		cvCvtColor(input2, gray2, CV_BGRA2GRAY);

		// call tracking algorithm
		tracker1->UpdateCameraPose(gray1);
		tracker2->UpdateCameraPose(gray2);

		tracker1->DrawDebugInfo(input1);
		tracker2->DrawDebugInfo(input2);

		tracker1->DrawInfomation(input1, 100.0);
		tracker2->DrawInfomation(input2, 100.0);

		// draw spatial position
		for(int i=0; i<spatialSensors.size(); i++)
		{
			if(spatialSensors[i]->GetPosition().z == 0)
			{
				cvCircle(input1, tracker1->GetCameraParameter()->ConvertWorld2Image(spatialSensors[i]->GetPosition().x, spatialSensors[i]->GetPosition().y, spatialSensors[i]->GetPosition().z), 2, CV_RGB(255, 0, 0), 1);
				cvCircle(input2, tracker2->GetCameraParameter()->ConvertWorld2Image(spatialSensors[i]->GetPosition().x, spatialSensors[i]->GetPosition().y, spatialSensors[i]->GetPosition().z), 2, CV_RGB(0, 255, 0), 1);
			}
		}

		std::vector<IplImage*> images;
		images.push_back(gray1);
		images.push_back(gray2);

		cvShowImage("image1", input1);
		cvShowImage("image2", input2);

		if(isSpatialProcessing)
		{
			std::cout << "start processing" << std::endl;
			cvZero(spatialImage);

			int count = 0;
//			#pragma omp parallel for
			for(int y=0; y<SPATIAL_Y; y++)
			{
				for(int x=0; x<SPATIAL_X; x++)
				{
					int minDistanceIndex = -1;
					double minDistance = 100.0;
					double distance = 0.0;
					for(int z=0; z<SPATIAL_Z; z++)
					{
//						#pragma omp critical
						{
							distance = ((windage::SPATIAL_SENSOR_TYPE*)spatialSensors[z*(SPATIAL_X*SPATIAL_Y) + y*(SPATIAL_X) + x])->GetDisparity(&images);
						}
						if(minDistance > distance)
						{
							minDistance = distance;
							minDistanceIndex = z;
						}
					}
					
//					#pragma omp critical
					{
						count++;
						cvSetReal2D(spatialImage, y, x, minDistanceIndex * (256/SPATIAL_Z));
						std::cout << "processing : " << ((double)(count) / (double)(SPATIAL_X*SPATIAL_Y)) * 100.0<< "%" << std::endl;
					}
				}
			}

			cvShowImage("spatial", spatialImage);
			isSpatialProcessing = false;

			cvSaveImage("spatial.jpg", spatialImage);
			cvSaveImage("input1.jpg", input1);
			cvSaveImage("input2.jpg", input2);

			std::cout << "end processing" << std::endl;
		}

		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 'p':
		case 'P':
			isSpatialProcessing = !isSpatialProcessing;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}
	}

	camera1->stop();
	camera1->close();
	camera2->stop();
	camera2->close();
}