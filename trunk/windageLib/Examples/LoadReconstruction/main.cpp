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

//const char* FILE_NAME = "data/reconstruction_2010-03-05_09_10_47.txt";
//const char* FILE_NAME = "data/reconstruction_2010-03-05_15_18_25.txt";
//const char* FILE_NAME = "data/reconstruction_2010-03-11_17_11_49.txt";
const char* FILE_NAME = "data/reconstruction_2010-03-12_09_36_30.txt";

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;

void main()
{
	std::vector<IplImage*> inputImage;
	std::vector<windage::ReconstructionPoint> reconstructionPoints;
	std::vector<std::string> filenameList;
	std::vector<IplImage*> imageList;
	std::vector<windage::Calibration*> calibrationList;

	int selectedCamera = -1;

	windage::Logger* logging;

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
	renderingSceneNode->SetCalibrationList(&calibrationList);
	renderingSceneNode->SetReconstructionPoints(&reconstructionPoints);
	renderingSceneNode->SetFileNameList(&filenameList);

	renderingSceneNode->Initialize();

	while(device->run())
	if (device->isWindowActive())
	{
		driver->beginScene(true, true, irr::video::SColor(0,100,100,100));
		smgr->drawAll();
		driver->endScene();

		if(receiver.IsKeyDown(irr::KEY_KEY_Q))
		{
			selectedCamera = -1;
			smgr->addCameraSceneNodeMaya();

			renderingSceneNode->setSelectedCamera();
			renderingSceneNode->resetTransparent();
		}
		else if(receiver.IsKeyDown(irr::KEY_KEY_A))
		{
			selectedCamera++;
			if(selectedCamera >= (int)calibrationList.size())
				selectedCamera = 0;

			CvScalar pt = calibrationList[selectedCamera]->GetCameraPosition();
			CvScalar at = calibrationList[selectedCamera]->GetLookAt();
			CvScalar up = calibrationList[selectedCamera]->GetUpPoint();

			windage::Vector3 cameraPosition	= windage::Vector3(pt.val[0], pt.val[1], pt.val[2]);
			windage::Vector3 lookAt			= windage::Vector3(at.val[0], at.val[1], at.val[2]);
			windage::Vector3 upVector		= windage::Vector3(up.val[0], up.val[1], up.val[2]);

			upVector -= cameraPosition;
			upVector /= upVector.getLength();

			smgr->addCameraSceneNode(0, irr::core::vector3df(cameraPosition.x, cameraPosition.y, cameraPosition.z), 
										irr::core::vector3df(lookAt.x, lookAt.y, lookAt.z));
			smgr->getActiveCamera()->setUpVector(irr::core::vector3df(upVector.x, upVector.y, upVector.z));

			renderingSceneNode->setSelectedCamera(selectedCamera);
			renderingSceneNode->setTransparent();
		}
		else if(receiver.IsKeyDown(irr::KEY_KEY_Z))
		{
			selectedCamera--;
			if(0 > selectedCamera)
				selectedCamera = (int)calibrationList.size() - 1;

			CvScalar pt = calibrationList[selectedCamera]->GetCameraPosition();
			CvScalar at = calibrationList[selectedCamera]->GetLookAt();
			CvScalar up = calibrationList[selectedCamera]->GetUpPoint();

			windage::Vector3 cameraPosition	= windage::Vector3(pt.val[0], pt.val[1], pt.val[2]);
			windage::Vector3 lookAt			= windage::Vector3(at.val[0], at.val[1], at.val[2]);
			windage::Vector3 upVector		= windage::Vector3(up.val[0], up.val[1], up.val[2]);

			upVector -= cameraPosition;
			upVector /= upVector.getLength();

			smgr->addCameraSceneNode(0, irr::core::vector3df(cameraPosition.x, cameraPosition.y, cameraPosition.z), 
										irr::core::vector3df(lookAt.x, lookAt.y, lookAt.z));
			smgr->getActiveCamera()->setUpVector(irr::core::vector3df(upVector.x, upVector.y, upVector.z));

			renderingSceneNode->setSelectedCamera(selectedCamera);
			renderingSceneNode->setTransparent();
		}
	}

	device->drop();
}
