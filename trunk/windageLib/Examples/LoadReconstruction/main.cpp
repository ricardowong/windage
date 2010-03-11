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
#include "../Common/OpenGLRenderer.h"

//const char* FILE_NAME = "data/reconstruction_2010-03-05_09_10_47.txt";
//const char* FILE_NAME = "data/reconstruction_2010-03-05_15_18_25.txt";
const char* FILE_NAME = "data/reconstruction_2010-03-11_17_11_49.txt";

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;
const int RENDERING_WIDTH = 640;
const int RENDERING_HEIGHT = (RENDERING_WIDTH * 3) / 4;

std::vector<IplImage*> inputImage;
std::vector<windage::ReconstructionPoint> reconstructionPoints;
std::vector<std::string> filenameList;
std::vector<IplImage*> imageList;
std::vector<windage::Calibration*> calibrationList;

int selectedCamera = -1;

windage::Logger* logging;

class MyEventReceiver : public irr::IEventReceiver
{
public:
	// This is the one method that we have to implement
	virtual bool OnEvent(const irr::SEvent& event)
	{
		// Remember whether each key is down or up
		if (event.EventType == irr::EET_KEY_INPUT_EVENT)
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

		return false;
	}

	// This is used to check whether a key is being held down
	virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const
	{
		return KeyIsDown[keyCode];
	}
	
	MyEventReceiver()
	{
		for (irr::u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
			KeyIsDown[i] = false;
	}

private:
	// We use this array to store the current state of each key
	bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];
};


class CRenderSceneNode : public irr::scene::ISceneNode
{
	int size;
	irr::core::aabbox3d<irr::f32> Box;
	irr::video::S3DVertex* Vertices;
	std::vector<std::vector<irr::core::vector3df>> cameras;
	std::vector<irr::video::SMaterial> cameraImages;

	irr::video::SMaterial Material;
public:

	CRenderSceneNode(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
		: irr::scene::ISceneNode(parent, mgr, id)
	{
		this->Material.Wireframe = false;
		this->Material.Lighting = false;

		// draw reconstruction points
		this->size = (int)reconstructionPoints.size();
		this->Vertices = new irr::video::S3DVertex[size];
		for(int i=0; i<size; i++)
		{
			windage::ReconstructionPoint point = reconstructionPoints[i];
			windage::Vector4 position = point.GetPoint();
			CvScalar color = point.GetColor();
			Vertices[i] = irr::video::S3DVertex(position.x, position.y, position.z, 1, 1, 1, irr::video::SColor(255, color.val[2], color.val[1], color.val[0]), 0, 0);
		}

		// draw camera frame
		int count = (int)calibrationList.size();
		this->cameras.resize(count);
		this->cameraImages.resize(count);
		for(int i=0; i<count; i++)
		{
			windage::Calibration* calibration = calibrationList[i];

			CvScalar pt = calibration->GetCameraPosition();
			CvScalar at = calibration->GetLookAt();
			CvScalar up = calibration->GetUpPoint();
			CvScalar ri = calibration->GetRightPoint();

			windage::Vector3 cameraPosition	= windage::Vector3(pt.val[0], pt.val[1], pt.val[2]);
			windage::Vector3 lookAt			= windage::Vector3(at.val[0], at.val[1], at.val[2]);
			windage::Vector3 upVector		= windage::Vector3(up.val[0], up.val[1], up.val[2]);
			windage::Vector3 rightVector	= windage::Vector3(ri.val[0], ri.val[1], ri.val[2]);
			windage::Vector3 point[4];

			// translate relation value
			lookAt -= cameraPosition;
			upVector -= cameraPosition;
			rightVector -= cameraPosition;
			
			lookAt *= ((double)calibration->GetParameters()[0]) / lookAt.getLength();
			upVector *= ((double)HEIGHT/2.0) / upVector.getLength();
			rightVector *= ((double)WIDTH/2.0) / rightVector.getLength();

			double scale = 0.05;
			lookAt *= scale;
			upVector *= scale;
			rightVector *= scale;

			point[0] = cameraPosition + lookAt + upVector - rightVector;
			point[1] = cameraPosition + lookAt + upVector + rightVector;
			point[2] = cameraPosition + lookAt - upVector + rightVector;
			point[3] = cameraPosition + lookAt - upVector - rightVector;

			CvScalar color = CV_RGB(255, 255, 0);

			this->cameras[i].resize(5);
			this->cameras[i][0] = irr::core::vector3df(cameraPosition.x, cameraPosition.y, cameraPosition.z);
			for(int k=0; k<4; k++)
				this->cameras[i][k+1] = irr::core::vector3df(point[k].x, point[k].y, point[k].z);

			// load texture image
			irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();
			this->cameraImages[i].setTexture(0, driver->getTexture(filenameList[i].c_str()));
			this->cameraImages[i].Lighting = false;
			this->cameraImages[i].Wireframe = false;
			this->cameraImages[i].FrontfaceCulling = false;
			
		}
	}
	~CRenderSceneNode()
	{
		delete Vertices;
	}

	virtual void OnRegisterSceneNode()
	{
		if (IsVisible)
			SceneManager->registerNodeForRendering(this);

		ISceneNode::OnRegisterSceneNode();
	}

	void setTransparent()
	{
		for(int i=0; i<(int)calibrationList.size(); i++)
			this->cameraImages[i].MaterialType = irr::video::EMT_TRANSPARENT_ADD_COLOR;
	}

	void resetTransparent()
	{
		for(int i=0; i<(int)calibrationList.size(); i++)
			this->cameraImages[i].MaterialType = irr::video::EMT_SOLID;
	}

	virtual void render()
	{
		irr::u32* indices = new irr::u32[size];
		for(int i=0; i<size; i++)
			indices[i] = i;
		irr::u16 cameraIndices1[] = {0,1,2, 2,3,0};
		irr::u16 cameraIndices2[] = {0,2,1, 2,0,3};

		irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();

		driver->setMaterial(Material);
		driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);
		driver->drawVertexPrimitiveList(&Vertices[0], size, &indices[0], size, irr::video::EVT_STANDARD, irr::scene::EPT_POINTS, irr::video::EIT_32BIT);

		if(selectedCamera < 0)
		{
			for(unsigned int i=0; i<this->cameras.size(); i++)
			{
				driver->draw3DLine(this->cameras[i][1], this->cameras[i][2], irr::video::SColor(255, 255, 255, 0));
				driver->draw3DLine(this->cameras[i][2], this->cameras[i][3], irr::video::SColor(255, 255, 255, 0));
				driver->draw3DLine(this->cameras[i][3], this->cameras[i][4], irr::video::SColor(255, 255, 255, 0));
				driver->draw3DLine(this->cameras[i][4], this->cameras[i][1], irr::video::SColor(255, 255, 255, 0));
				for(int k=0; k<4; k++)
					driver->draw3DLine(this->cameras[i][0], this->cameras[i][k+1], irr::video::SColor(255, 255, 255, 0));
			}

			for(unsigned int i=0; i<this->cameras.size(); i++)
			{
				irr::video::S3DVertex cameraVertices[4];

				driver->setMaterial(this->cameraImages[i]);
				driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);

				cameraVertices[0] = irr::video::S3DVertex(this->cameras[i][1].X,this->cameras[i][1].Y,this->cameras[i][1].Z, 1,1,1, irr::video::SColor(255,255,255,255), 0, 0);
				cameraVertices[1] = irr::video::S3DVertex(this->cameras[i][2].X,this->cameras[i][2].Y,this->cameras[i][2].Z, 1,1,1, irr::video::SColor(255,255,255,255), 1, 0);
				cameraVertices[2] = irr::video::S3DVertex(this->cameras[i][3].X,this->cameras[i][3].Y,this->cameras[i][3].Z, 1,1,1, irr::video::SColor(255,255,255,255), 1, 1);
				cameraVertices[3] = irr::video::S3DVertex(this->cameras[i][4].X,this->cameras[i][4].Y,this->cameras[i][4].Z, 1,1,1, irr::video::SColor(255,255,255,255), 0, 1);
				driver->drawVertexPrimitiveList(&cameraVertices[0], 4, &cameraIndices1[0], 2, irr::video::EVT_STANDARD, irr::scene::EPT_TRIANGLES, irr::video::EIT_16BIT);
				driver->drawVertexPrimitiveList(&cameraVertices[0], 4, &cameraIndices2[0], 2, irr::video::EVT_STANDARD, irr::scene::EPT_TRIANGLES, irr::video::EIT_16BIT);
			}
		}
		else
		{
			int i = selectedCamera;
			driver->draw3DLine(this->cameras[i][1], this->cameras[i][2], irr::video::SColor(255, 255, 255, 0));
			driver->draw3DLine(this->cameras[i][2], this->cameras[i][3], irr::video::SColor(255, 255, 255, 0));
			driver->draw3DLine(this->cameras[i][3], this->cameras[i][4], irr::video::SColor(255, 255, 255, 0));
			driver->draw3DLine(this->cameras[i][4], this->cameras[i][1], irr::video::SColor(255, 255, 255, 0));
			for(int k=0; k<4; k++)
				driver->draw3DLine(this->cameras[i][0], this->cameras[i][k+1], irr::video::SColor(255, 255, 255, 0));

			irr::video::S3DVertex cameraVertices[4];

				driver->setMaterial(this->cameraImages[i]);
			driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);

			int value = 200;
			cameraVertices[0] = irr::video::S3DVertex(this->cameras[i][1].X,this->cameras[i][1].Y,this->cameras[i][1].Z, 1,1,1, irr::video::SColor(255, value,value,value), 0, 0);
			cameraVertices[1] = irr::video::S3DVertex(this->cameras[i][2].X,this->cameras[i][2].Y,this->cameras[i][2].Z, 1,1,1, irr::video::SColor(255, value,value,value), 1, 0);
			cameraVertices[2] = irr::video::S3DVertex(this->cameras[i][3].X,this->cameras[i][3].Y,this->cameras[i][3].Z, 1,1,1, irr::video::SColor(255, value,value,value), 1, 1);
			cameraVertices[3] = irr::video::S3DVertex(this->cameras[i][4].X,this->cameras[i][4].Y,this->cameras[i][4].Z, 1,1,1, irr::video::SColor(255, value,value,value), 0, 1);
			driver->drawVertexPrimitiveList(&cameraVertices[0], 4, &cameraIndices1[0], 2, irr::video::EVT_STANDARD, irr::scene::EPT_TRIANGLES, irr::video::EIT_16BIT);
			driver->drawVertexPrimitiveList(&cameraVertices[0], 4, &cameraIndices2[0], 2, irr::video::EVT_STANDARD, irr::scene::EPT_TRIANGLES, irr::video::EIT_16BIT);
		}

		delete indices;
	}

	virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const
	{
		return Box;
	}

	virtual irr::u32 getMaterialCount() const
	{
		return 1;
	}

	virtual irr::video::SMaterial& getMaterial(irr::u32 i)
	{
		return Material;
	}	
};

int main()
{
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
	MyEventReceiver receiver;
	irr::IrrlichtDevice* device = irr::createDevice( irr::video::EDT_DIRECT3D9, irr::core::dimension2d<irr::u32>(640, 480), 16, false, false, false, &receiver);
	if (!device) return 1;

	device->setWindowCaption(L"Spatial Reconstruction");

	irr::video::IVideoDriver* driver = device->getVideoDriver();
	irr::scene::ISceneManager* smgr = device->getSceneManager();
	irr::scene::ICameraSceneNode* camera = smgr->addCameraSceneNodeMaya();
	camera->setPosition(irr::core::vector3df(100, 100, 100));

	CRenderSceneNode *myNode = new CRenderSceneNode(smgr->getRootSceneNode(), smgr, 666);
//	myNode->drop();
//	myNode = 0;

	irr::u32 frames=0;
	while(device->run())
	if (device->isWindowActive())
	{
		driver->beginScene(true, true, irr::video::SColor(0,100,100,100));
		smgr->drawAll();
		driver->endScene();

		if (++frames==100)
		{
			irr::core::stringw str = L"windage : Spatial Reconstruction [";
			str += driver->getName();
			str += L"] FPS: ";
			str += (irr::s32)driver->getFPS();

			device->setWindowCaption(str.c_str());
			frames=0;
		}

		if(receiver.IsKeyDown(irr::KEY_KEY_Q))
		{
			selectedCamera = -1;
			smgr->addCameraSceneNodeMaya();

			myNode->resetTransparent();
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
			myNode->setTransparent();
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
			myNode->setTransparent();
		}
	}

	device->drop();
	return 0;
}
