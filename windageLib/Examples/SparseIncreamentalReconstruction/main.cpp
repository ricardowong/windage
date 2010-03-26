/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
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
#include <direct.h>
#include <omp.h>

#include <cv.h>
#include <highgui.h>
#include <irrlicht.h>

#include <windage.h>
#include "../Common/IrrlichtRenderer.h"

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;
const double INTRINSIC_VALUES[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

const char* RECONSTRUCTION_PATH_TEMPLATE = "data/reconstruction-%s";
const char* RECONSTRUCTION_FILENAME = "%s/reconstruction";
const char* IMAGE_FILE_NAME_TEMPLATE = "%s/image%03d.png";

const double RANSAC_COEFFICIENT = 0.99995;
const int RANSAC_ITERATION = 5000;
const double RANSAC_REPROJECTION_ERROR = 2.0;
const double RECONSTRUCTION_SIZE = 50.0;
const int BUNDLEADSUTMENT_COUNT = 3;

void main()
{
	windage::Logger logger(&std::cout);
	logger.updateTickCount();

	// camera capture
	IplImage* inputImage;
	IplImage* resizeImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvNamedWindow("result");

	// feature extractor
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();
	std::vector<IplImage*> reconstructionImages;

	// for reconstruction
	windage::Calibration* initialCalibration = new windage::Calibration();
	initialCalibration->Initialize(INTRINSIC_VALUES[0], INTRINSIC_VALUES[1], INTRINSIC_VALUES[2], INTRINSIC_VALUES[3]);

	windage::Reconstruction::IncrementalReconstruction* reconstructor = new windage::Reconstruction::IncrementalReconstruction();
	reconstructor->SetConfidence(RANSAC_COEFFICIENT);
	reconstructor->SetMaxIteration(RANSAC_ITERATION);
	reconstructor->SetReprojectionError(RANSAC_REPROJECTION_ERROR);

	reconstructor->AttatchCalibration(initialCalibration);

	windage::Algorithms::SearchTree* tree = new windage::Algorithms::KDtree(50);
	tree->SetRatio(0.6);
	reconstructor->AttatchSearchTree(tree);

	windage::Algorithms::OpenCVRANSACestimator* estimator = new windage::Algorithms::OpenCVRANSACestimator();
	estimator->SetConfidence(RANSAC_COEFFICIENT);
	estimator->SetMaxIteration(RANSAC_ITERATION);
	estimator->SetReprojectionError(RANSAC_REPROJECTION_ERROR);
	reconstructor->AttatchEstimator(estimator);

	// reconstruction data
	std::vector<std::vector<windage::FeaturePoint>> featurePoint;
	std::vector<std::string> filenameList;

	// for rendering
	KeyEventReceiver receiver;
	irr::IrrlichtDevice* device = irr::createDevice( irr::video::EDT_DIRECT3D9, irr::core::dimension2d<irr::u32>(640, 480), 16, false, false, false, &receiver);
	if (!device) return;
	device->setWindowCaption(L"windage : Spatial Reconstruction");

	irr::video::IVideoDriver* driver = device->getVideoDriver();
	irr::scene::ISceneManager* smgr = device->getSceneManager();
	irr::scene::ICameraSceneNode* camera = smgr->addCameraSceneNodeMaya();
	camera->setPosition(irr::core::vector3df(100, 100, 100));

	SceneNode *renderingSceneNode = new SceneNode(smgr->getRootSceneNode(), smgr, 666);

	int imageCount = 0;
	char message[100];

	char beforeCh = -1;
	bool flip = true;
	bool processing = true;
	while(processing && device->run())
	{
		// capture image
		inputImage = cvRetrieveFrame(capture);
		if(flip)
			cvFlip(inputImage, inputImage);
		cvResize(inputImage, resizeImage);
		cvCvtColor(resizeImage, grayImage, CV_BGR2GRAY);
		cvCopyImage(resizeImage, resultImage);

		double processingTime = logger.calculateProcessTime();
//		logger.log(processingTime); logger.logNewLine();
		logger.updateTickCount();

		sprintf_s(message, "Processing Time : %.2lf ms", processingTime);
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, 20), 0.6, message);
		sprintf_s(message, "Press 'F' to flip image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, HEIGHT-40), 0.6, message);
		sprintf_s(message, "Press 'S' to save reconstruction datas");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, HEIGHT-25), 0.6, message);
		sprintf_s(message, "Press 'Space' or 'a' to add reconstruction current image");
		windage::Utils::DrawTextToImage(resultImage, cvPoint(10, HEIGHT-10), 0.6, message);
		
		cvShowImage("result", resultImage);

		// render reconstruction data
		driver->beginScene(true, true, irr::video::SColor(0,100,100,100));
		smgr->drawAll();
		driver->endScene();

		char ch = cvWaitKey(1);
		if(ch == -1)
		{
			ch = receiver.GetDownKeycode();
		}
		if(beforeCh == ch)
		{
			ch = -1;
		}
		else
		{
			beforeCh = ch;
		}

		switch(ch)
		{
		case 27:
		case 'q':
		case 'Q':
			processing = false;
			break;
		case 'f':
		case 'F':
			flip = !flip;
			break;
		case 's':
		case 'S':
			{
				std::string timeString = windage::Logger::getTimeString();

				// create folder
				char path[100];
				sprintf_s(path, RECONSTRUCTION_PATH_TEMPLATE, timeString.c_str());
				_mkdir(path);

				// save data
				std::cout << "save reconstruction datas" << std::endl;
				windage::Reconstruction::Exportor exportor;

				sprintf_s(message, RECONSTRUCTION_FILENAME, path);
				windage::Logger* reconstructionLogger = new windage::Logger(message, "txt");

				exportor.SetFunctionName(detector->GetFunctionName());
				exportor.AttatchLogger(reconstructionLogger);
				exportor.SetReconstructionPoints(reconstructor->GetReconstructedPoint());
				for(int i=0; i<imageCount; i++)
				{
					sprintf_s(message, IMAGE_FILE_NAME_TEMPLATE, path, i);
					cvSaveImage(message, reconstructionImages[i]);

					exportor.PushCalibration(reconstructor->GetCameraParameter(i));
					exportor.PushImageFile(message);
				}
				exportor.DoExport();
				delete reconstructionLogger;
				reconstructionLogger = NULL;
			}
			break;
		case ' ':
		case 'a':
		case 'A':
			{
				std::cout << "add reconstruction image" << std::endl;
				featurePoint.resize(imageCount+1);

				detector->DoExtractKeypointsDescriptor(grayImage);
				std::vector<windage::FeaturePoint>* tempKeypoints = detector->GetKeypoints();
				for(unsigned int j=0; j<tempKeypoints->size(); j++)
				{
					(*tempKeypoints)[j].SetColor(cvGet2D(resizeImage, cvRound((*tempKeypoints)[j].GetPoint().y), cvRound((*tempKeypoints)[j].GetPoint().x)));
					featurePoint[imageCount].push_back((*tempKeypoints)[j]);
				}

				reconstructor->AttatchFeaturePoint(&featurePoint[imageCount]);
				reconstructionImages.push_back(cvCloneImage(resizeImage));

				imageCount++;
				if(imageCount >= 2)
				{
					reconstructor->CalculateStep(imageCount);

					int startIndex = MAX(0, imageCount - BUNDLEADSUTMENT_COUNT);
					int countNumber = MIN(imageCount, BUNDLEADSUTMENT_COUNT);
					reconstructor->BundleAdjustment(startIndex, countNumber);

					reconstructor->ResizeScale(RECONSTRUCTION_SIZE);

					renderingSceneNode->SetCalibrationList(reconstructor->GetCameraParameterList());
					renderingSceneNode->SetReconstructionPoints(reconstructor->GetReconstructedPoint());
//					renderingSceneNode->SetFileNameList(&filenameList);

					renderingSceneNode->Initialize();
				}
			}
			break;
		case 'b':
		case 'B':
			{
				std::cout << "bundle adjustment all points" << std::endl;
				reconstructor->BundleAdjustment();
			}
			break;
		case 'r':
		case 'R':
			{
				std::cout << "remove reconstruction image" << std::endl;
			}
			break;
		}		
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
