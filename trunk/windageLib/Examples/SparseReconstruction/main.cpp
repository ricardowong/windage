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
#include <omp.h>

#include <cv.h>
#include <highgui.h>
#include <irrlicht.h>

#include <windage.h>
#include "../Common/IrrlichtRenderer.h"

#define RECONSTRUCTION_TEST 1
#define RECONSTRUCTION_TEMPLE 0
#define RECONSTRUCTION_MINIATURE 0

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;

#if RECONSTRUCTION_TEST
const int START_INDEX = 0;
const int IMAGE_FILE_COUNT = 10;
const char* IMAGE_FILE_NAME = "Reconstruction/test%03d.jpg";
const double INTRINSIC_VALUES[8] = {WIDTH*0.8, WIDTH*0.8, WIDTH/2, HEIGHT/2, 0, 0, 0, 0};
#endif

#if RECONSTRUCTION_TEMPLE
const int START_INDEX = 1;
const int IMAGE_FILE_COUNT = 45;
const double INTRINSIC_VALUES[8] = {1520.40, 1525.90, 302.32, 246.87, 0, 0, 0, 0};
const char* IMAGE_FILE_NAME = "templeOrder/templeR%04d.png";
#endif

#if RECONSTRUCTION_MINIATURE
const int START_INDEX = 1;
const int IMAGE_FILE_COUNT = 79;
const double INTRINSIC_VALUES[8] = {608.894958, 609.015991, 295.023712, 254.171387, 0, 0, 0, 0};
const char* IMAGE_FILE_NAME = "Miniature/result.imgr%d_COL.jpg";
#endif

const double RANSAC_COEFFICIENT = 0.99995;
const int RANSAC_ITERATION = 5000;
const double RANSAC_REPROJECTION_ERROR = 2.0;

double SCALE = 100.0;

void main()
{
	std::vector<IplImage*> inputImage;

	windage::Calibration* initialCalibration;
	windage::Reconstruction::IncrementalReconstruction* reconstructor;
	std::vector<std::string> filenameList;
	windage::Logger* logging;

	// reconstruction
	std::vector<IplImage*> grayImage;
	inputImage.resize(IMAGE_FILE_COUNT);
	grayImage.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> featurePoint;
	featurePoint.resize(IMAGE_FILE_COUNT);

	std::vector<std::vector<windage::FeaturePoint>> matchedPoint;
	matchedPoint.resize((IMAGE_FILE_COUNT-1)*2);

	logging = new windage::Logger(&std::cout);
	logging->log("initialize"); logging->logNewLine();

	initialCalibration = new windage::Calibration();
	initialCalibration->Initialize(INTRINSIC_VALUES[0], INTRINSIC_VALUES[1], INTRINSIC_VALUES[2], INTRINSIC_VALUES[3]);

	reconstructor = new windage::Reconstruction::IncrementalReconstruction();
	reconstructor->SetConfidence(RANSAC_COEFFICIENT);
	reconstructor->SetMaxIteration(RANSAC_ITERATION);
	reconstructor->SetReprojectionError(RANSAC_REPROJECTION_ERROR);

	reconstructor->AttatchCalibration(initialCalibration);

	windage::Algorithms::SearchTree* tree = new windage::Algorithms::FLANNtree(50);
	tree->SetRatio(0.3);
	reconstructor->AttatchSearchTree(tree);

//	windage::Algorithms::EPnPRANSACestimator* estimator = new windage::Algorithms::EPnPRANSACestimator();
	windage::Algorithms::OpenCVRANSACestimator* estimator = new windage::Algorithms::OpenCVRANSACestimator();
	estimator->SetConfidence(RANSAC_COEFFICIENT);
	estimator->SetMaxIteration(RANSAC_ITERATION);
	estimator->SetReprojectionError(RANSAC_REPROJECTION_ERROR);
	reconstructor->AttatchEstimator(estimator);

	logging->logNewLine();
	logging->log("load image & feature extract - matching"); logging->logNewLine();

	char filename[100];
	int index = 0;
	for(int i=START_INDEX; i<IMAGE_FILE_COUNT+START_INDEX; i++)
	{
		sprintf_s(filename, IMAGE_FILE_NAME, i);
		filenameList.push_back(std::string(filename));
		inputImage[index] = cvLoadImage(filename);
		grayImage[index] = cvCreateImage(cvGetSize(inputImage[index]), IPL_DEPTH_8U, 1);

		cvCvtColor(inputImage[index], grayImage[index], CV_BGR2GRAY);

		logging->log("\tload image "); logging->log(index); logging->log(" : "); logging->log(filename); logging->logNewLine();
		index++;
	}

	logging->logNewLine();
	logging->log("\tfeature extract"); logging->logNewLine();

//	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTdetector();
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		detector->DoExtractKeypointsDescriptor(grayImage[i]);
		std::vector<windage::FeaturePoint>* temp = detector->GetKeypoints();

		featurePoint[i].clear();
		for(unsigned int j=0; j<temp->size(); j++)
		{
			(*temp)[j].SetColor(cvGet2D(inputImage[i], (*temp)[j].GetPoint().y, (*temp)[j].GetPoint().x));
			featurePoint[i].push_back((*temp)[j]);
		}

		logging->log("\tkeypoint count "); logging->log(i); logging->log(" : "); logging->log((int)featurePoint[i].size()); logging->logNewLine();
	}

	logging->logNewLine();
	logging->log("reconstruction sequence"); logging->logNewLine();
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		reconstructor->AttatchFeaturePoint(&featurePoint[i]);
	}
	reconstructor->CalculateAll();
	reconstructor->UpdateColor();
	reconstructor->ResizeScale(SCALE);

	std::cout << std::endl;

	// save data
/*
	std::cout << "save reconstruction datas" << std::endl;
	windage::Reconstruction::Exportor exportor;
	windage::Logger* reconstructionLogger = new windage::Logger("data/reconstruction", "txt", true);
	exportor.SetFunctionName(detector->GetFunctionName());
	exportor.AttatchLogger(reconstructionLogger);
	exportor.SetReconstructionPoints(reconstructor->GetReconstructedPoint());
	for(int i=0; i<IMAGE_FILE_COUNT; i++)
	{
		exportor.PushCalibration(reconstructor->GetCameraParameter(i));
		
		sprintf_s(filename, IMAGE_FILE_NAME, i+START_INDEX);
		exportor.PushImageFile(std::string(filename));
	}
	exportor.DoExport();
	delete reconstructionLogger;
	reconstructionLogger = NULL;

	std::cout << std::endl;
//*/
	
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
	renderingSceneNode->SetCalibrationList(reconstructor->GetCameraParameterList());
	renderingSceneNode->SetReconstructionPoints(reconstructor->GetReconstructedPoint());
	renderingSceneNode->SetFileNameList(&filenameList);

	renderingSceneNode->Initialize();

	while(device->run())
	if (device->isWindowActive())
	{
		driver->beginScene(true, true, irr::video::SColor(0,100,100,100));
		smgr->drawAll();
		driver->endScene();
	}

	device->drop();
}
