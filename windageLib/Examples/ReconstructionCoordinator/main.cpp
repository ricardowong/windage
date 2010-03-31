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
#include "Coordinator/ARForOpenGL.h"
#include "Coordinator/ARForOSG.h"
#include "../Common/IrrlichtRenderer.h"

const char* FILE_NAME = "data/reconstruction-2010-03-29_18_28_38/reconstruction.txt";
const char* COORDINATION_ALIGN_IMAGE = "data/reconstruction-2010-03-29_18_28_38/coordination.jpg";
const char* MODEL_FILE_NAME = "data/Model/Tank/Tank.obj";
const char* MODEL_TEXTURE_FILE_NAME = "data/Model/Tank/images/M1_ABRAM.png";
const double MODEL_SCALE = 0.1;

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;

const double REPROJECTION_ERROR = 10.0;
const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

bool Matching(windage::Algorithms::SearchTree* searchtree, std::vector<windage::FeaturePoint>* feature1, std::vector<windage::FeaturePoint>* feature2, std::vector<windage::FeaturePoint>* matchedPoint1, std::vector<windage::FeaturePoint>* matchedPoint2)
{
	searchtree->Training(feature1);
	for(unsigned int i=0; i<feature2->size(); i++)
	{
		int index = searchtree->Matching((*feature2)[i]);
		if(index >= 0)
		{
			matchedPoint1->push_back((*feature1)[index]);
			matchedPoint2->push_back((*feature2)[i]);
			(*matchedPoint1)[matchedPoint1->size()-1].SetRepositoryID(index);
			(*matchedPoint2)[matchedPoint1->size()-1].SetRepositoryID(i);
		}
	}
	return true;
}

void main()
{
	std::vector<IplImage*> inputImage;
	std::vector<windage::ReconstructionPoint> reconstructionPoints;
	std::vector<std::string> filenameList;
	std::vector<IplImage*> imageList;
	std::vector<windage::Calibration*> calibrationList;

	int selectedCamera = -1;

	// load data
	std::cout << "load reconstruction datas" << std::endl;
	std::cout << std::endl;

	windage::Reconstruction::Loader* loader = new windage::Reconstruction::Loader();
	loader->AttatchCalibration(&calibrationList);
	loader->AttatchFilename(&filenameList);
	loader->AttatchReconstructionPoints(&reconstructionPoints);
	loader->DoLoad(FILE_NAME);

	for(unsigned int i=0; i<filenameList.size(); i++)
	{
		imageList.push_back(cvLoadImage(filenameList[i].c_str()));
	}

	std::cout << std::endl;
	std::cout << "complete load reconstruction datas" << std::endl;
	std::cout << "reconstruction data count : " << reconstructionPoints.size() << std::endl;
	std::cout << std::endl;

	// for tracking
	std::vector<windage::FeaturePoint> referenceRepository;
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		windage::FeaturePoint feature = reconstructionPoints[i].GetFeature(0);
		windage::Vector4 point = reconstructionPoints[i].GetPoint();
		feature.SetPoint(windage::Vector3(point.x, point.y, point.z));
		feature.SetObjectID(0);
		feature.SetRepositoryID(i);

		referenceRepository.push_back(feature);
	}

	windage::Calibration* calibrationTemp = new windage::Calibration();
	calibrationTemp->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	windage::Calibration* calibrationModel = new windage::Calibration();
	windage::Calibration* calibrationMarker = new windage::Calibration();
	calibrationModel->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	calibrationMarker->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);

	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::SIFTGPUdetector();
	windage::Algorithms::SearchTree* searchtree = new windage::Algorithms::FLANNtree();
	windage::Algorithms::OpenCVRANSACestimator* estimator = new windage::Algorithms::OpenCVRANSACestimator();
	windage::Algorithms::PoseRefiner* refiner = new windage::Algorithms::PoseLMmethod();

	windage::Algorithms::ChessboardDetector* baseDetector = new windage::Algorithms::ChessboardDetector(8, 7, 28);
	windage::Algorithms::HomographyEstimator* baseEstimator = new windage::Algorithms::LMedSestimator();

	IplImage* coordinationImage = cvLoadImage(COORDINATION_ALIGN_IMAGE);
	IplImage* grayImage = cvCreateImage(cvGetSize(coordinationImage), IPL_DEPTH_8U, 1);
	cvCvtColor(coordinationImage, grayImage, CV_BGR2GRAY);

	// model pose
	detector->DoExtractKeypointsDescriptor(grayImage);
	searchtree->Training(&referenceRepository);
	searchtree->SetRatio(0.5);
	std::vector<windage::FeaturePoint> match1;
	std::vector<windage::FeaturePoint> match2;

	Matching(searchtree, &referenceRepository, detector->GetKeypoints(), &match1, &match2);

	estimator->AttatchCameraParameter(calibrationModel);
	estimator->AttatchReferencePoint(&match1);
	estimator->AttatchScenePoint(&match2);
	estimator->SetReprojectionError(5.0);
	estimator->Calculate();

	int index = 0;
	int count = (int)match2.size();
	for(int i=0; i<count; i++)
	{
		if(match2[index].IsOutlier() == false)
		{
			windage::Vector3 point = match2[index].GetPoint();
			cvCircle(coordinationImage, cvPoint(point.x, point.y), 5, CV_RGB(0, 0, 0), CV_FILLED);
			cvCircle(coordinationImage, cvPoint(point.x, point.y), 3, CV_RGB(255, 255, 0), CV_FILLED);
			index++;
		}
		else
		{
			windage::Vector3 point = match2[index].GetPoint();
			cvCircle(coordinationImage, cvPoint(point.x, point.y), 5, CV_RGB(0, 0, 0), CV_FILLED);
			cvCircle(coordinationImage, cvPoint(point.x, point.y), 3, CV_RGB(255, 0, 0), CV_FILLED);
			match1.erase(match1.begin() + index);
			match2.erase(match2.begin() + index);
		}
	}

	refiner->AttatchCalibration(calibrationModel);
	refiner->AttatchReferencePoint(&match1);
	refiner->AttatchScenePoint(&match2);

	calibrationModel->DrawInfomation(coordinationImage, 100.0);

	// marker pose
	baseDetector->FindMarker(grayImage);
	baseEstimator->AttatchCameraParameter(calibrationMarker);
	baseEstimator->AttatchReferencePoint(baseDetector->GetReferencePoints());
	baseEstimator->AttatchScenePoint(baseDetector->GetKeypoints());
	baseEstimator->Calculate();
	baseEstimator->DecomposeHomography();

	baseDetector->DrawMarkerInfo(coordinationImage);
	calibrationMarker->DrawInfomation(coordinationImage, 100.0);
	
	// convert coordination
	windage::Matrix3 rotation = windage::Coordinator::MultiMarkerCoordinator::GetRotation(calibrationModel, calibrationMarker);
	windage::Vector3 translation = windage::Vector3();
//	translation = windage::Coordinator::MultiMarkerCoordinator::GetTranslation(calibrationModel, calibrationMarker);

	windage::Matrix4 extrinsic = windage::Coordinator::MultiMarkerCoordinator::CalculateExtrinsic(calibrationModel, rotation, translation);
	calibrationTemp->SetExtrinsicMatrix(extrinsic.m1);
	
	cvNamedWindow("result");
	cvShowImage("result", coordinationImage);
	cvWaitKey();
	calibrationTemp->DrawInfomation(coordinationImage, 100.0);
	cvShowImage("result", coordinationImage);
	cvWaitKey();
	cvDestroyAllWindows();

	// apply result
	windage::Reconstruction::ConvertCoordination coordination;
	coordination.AttatchReconstructionPoint(&reconstructionPoints);
	coordination.ConvertRotation(rotation);
	coordination.ConvertTranslation(translation);
	for(unsigned int i=0; i<calibrationList.size(); i++)
	{
		windage::Matrix4 extrinsic = windage::Coordinator::MultiMarkerCoordinator::CalculateExtrinsic(calibrationList[i], rotation.Transpose(), translation);
		calibrationList[i]->SetExtrinsicMatrix(extrinsic.m1);
	}
	
	// for rendering
//*
	KeyEventReceiver receiver;
	irr::IrrlichtDevice* device = irr::createDevice(irr::video::EDT_DIRECT3D9, irr::core::dimension2d<irr::u32>(RENDERING_WIDTH, RENDERING_HEIGHT), 32, false, false, false, &receiver);
	if (!device) return;
	device->setWindowCaption(L"windage : Reconstruction Coordination");
	
	irr::video::IVideoDriver* driver = device->getVideoDriver();
	irr::scene::ISceneManager* smgr = device->getSceneManager();
	irr::scene::ICameraSceneNode* mayaCamera = smgr->addCameraSceneNodeMaya();
	irr::scene::ICameraSceneNode* arCamera = smgr->addCameraSceneNode();
/*
	irr::scene::ISceneNode* modelNode = smgr->addMeshSceneNode(smgr->getMesh(MODEL_FILE_NAME));
	modelNode->setMaterialTexture(0, driver->getTexture(MODEL_TEXTURE_FILE_NAME));
	modelNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);
	modelNode->setScale(irr::core::vector3df(MODEL_SCALE, MODEL_SCALE, MODEL_SCALE));
	smgr->setActiveCamera(mayaCamera);
*/

	SceneNode *renderingSceneNode = new SceneNode(smgr->getRootSceneNode(), smgr, 666);
	renderingSceneNode->SetCalibrationList(&calibrationList);
	renderingSceneNode->SetReconstructionPoints(&reconstructionPoints);
	renderingSceneNode->SetFileNameList(&filenameList);
	renderingSceneNode->Initialize();

	// coordination
	const double MOVEMENT_SPEED = 5.0;

	char beforeCh = -1;
	irr::u32 then = device->getTimer()->getTime();
	while(device->run())
	{
		driver->beginScene(true, true, irr::video::SColor(0,100,100,100));
		smgr->drawAll();
		driver->endScene();

		char ch = receiver.GetDownKeycode();
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
		case 'q':
		case 'Q':
			{
				smgr->setActiveCamera(mayaCamera);
				selectedCamera = -1;

				renderingSceneNode->setSelectedCamera();
				renderingSceneNode->resetTransparent();
			}
			break;
		case 'a':
		case 'A':
			{
				selectedCamera++;
				if(selectedCamera >= (int)calibrationList.size())
					selectedCamera = 0;
				windage::Coordinator::ARForOSG coordinator;
				coordinator.Initialize(RENDERING_WIDTH, RENDERING_HEIGHT);
				coordinator.AttatchCameraParameter(calibrationList[selectedCamera]);
				coordinator.SetProjectionMatrix();
				coordinator.SetModelViewMatrix();

				windage::Matrix4 projection = coordinator.GetProjectionMatrix();
				windage::Matrix4 modelview = coordinator.GetModelViewMatrix();

				irr::core::matrix4 irrProj;
				irr::core::matrix4 irrModel;
				for(int i=0; i<16; i++)
				{
					irrProj[i] = projection.m1[i];
					irrModel[i] = modelview.m1[i];
				}

				smgr->setActiveCamera(arCamera);
				smgr->getActiveCamera()->setProjectionMatrix(irrProj);
				smgr->getActiveCamera()->setViewMatrixAffector(irrModel);

				renderingSceneNode->setSelectedCamera(selectedCamera);
				renderingSceneNode->setTransparent();

			}
			break;
		case 'z':
		case 'Z':
			{
				selectedCamera--;
				if(0 > selectedCamera)
					selectedCamera = (int)calibrationList.size() - 1;

				windage::Coordinator::ARForOSG coordinator;
				coordinator.Initialize(RENDERING_WIDTH, RENDERING_HEIGHT);
				coordinator.AttatchCameraParameter(calibrationList[selectedCamera]);
				coordinator.SetProjectionMatrix();
				coordinator.SetModelViewMatrix();

				windage::Matrix4 projection = coordinator.GetProjectionMatrix();
				windage::Matrix4 modelview = coordinator.GetModelViewMatrix();

				irr::core::matrix4 irrProj;
				irr::core::matrix4 irrModel;
				for(int i=0; i<16; i++)
				{
					irrProj[i] = projection.m1[i];
					irrModel[i] = modelview.m1[i];
				}

				smgr->setActiveCamera(arCamera);
				smgr->getActiveCamera()->setProjectionMatrix(irrProj);
				smgr->getActiveCamera()->setViewMatrixAffector(irrModel);

				renderingSceneNode->setSelectedCamera(selectedCamera);
				renderingSceneNode->setTransparent();
			}
			break;
		}
	}

	device->drop();
//*/
}
