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

#include <vector>
#include <irrlicht.h>

#include <windage.h>

class KeyEventReceiver : public irr::IEventReceiver
{
public:
	// This is the one method that we have to implement
	virtual bool OnEvent(const irr::SEvent& event)
	{
		// Remember whether each key is down or up
		if (event.EventType == irr::EET_KEY_INPUT_EVENT)
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
		else if(event.EventType == irr::EET_MOUSE_INPUT_EVENT)
			KeyIsDown[event.MouseInput.Event] = event.MouseInput.Event;

		return false;
	}

	// This is used to check whether a key is being held down
	virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const
	{
		return KeyIsDown[keyCode];
	}
	
	KeyEventReceiver()
	{
		for (irr::u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
			KeyIsDown[i] = false;
	}

	char GetDownKeycode()
	{
		for (irr::u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
			if(KeyIsDown[i])
				return i;
		return -1;
	}

private:
	// We use this array to store the current state of each key
	bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];
};

class SceneNode : public irr::scene::ISceneNode
{
private:
	int size;
	irr::core::aabbox3d<irr::f32> Box;
	irr::video::S3DVertex* Vertices;
	std::vector<std::vector<irr::core::vector3df>> cameras;
	std::vector<irr::video::SMaterial> cameraImages;

	irr::video::SMaterial Material;
	int selectedCamera;
	int width;
	int height;

	// reconstruction data
	std::vector<windage::ReconstructionPoint>* reconstructionPoints;
	std::vector<windage::Calibration*>* calibrationList;
	std::vector<std::string>* filenameList;

public:

	SceneNode(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
		: irr::scene::ISceneNode(parent, mgr, id)
	{
		Material.Wireframe = false;
		Material.Lighting = false;
		Material.Thickness = 3;

		size = 0;
		Vertices = NULL;
		reconstructionPoints = NULL;
		calibrationList = NULL;
		filenameList = NULL;

		selectedCamera = -1;
		this->width = 640;
		this->height = 480;
	}
	~SceneNode()
	{
		if(Vertices)
			delete Vertices;
	}

	inline void SetReconstructionPoints(std::vector<windage::ReconstructionPoint>* reconstructionPoints){this->reconstructionPoints = reconstructionPoints;}
	inline void SetCalibrationList(std::vector<windage::Calibration*>* calibrationList){this->calibrationList = calibrationList;}
	inline void SetFileNameList(std::vector<std::string>* filenameList){this->filenameList = filenameList;}
	inline void SetImageSize(int width, int height){this->width=width; this->height=height;}

	bool Initialize();
	virtual void render();

	virtual void OnRegisterSceneNode()
	{
		if (IsVisible)
			SceneManager->registerNodeForRendering(this);

		ISceneNode::OnRegisterSceneNode();
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

	void setTransparent()
	{
		for(unsigned int i=0; i<cameraImages.size(); i++)
		{
			this->cameraImages[i].MaterialType = irr::video::EMT_TRANSPARENT_ADD_COLOR;
		}
	}

	void resetTransparent()
	{
		for(unsigned int i=0; i<cameraImages.size(); i++)
		{
			this->cameraImages[i].MaterialType = irr::video::EMT_SOLID;
		}
	}

	void setSelectedCamera(int index = -1){this->selectedCamera = index;};
};
