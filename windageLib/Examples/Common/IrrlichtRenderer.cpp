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

#include "IrrlichtRenderer.h"

bool SceneNode::Initialize()
{
	this->Material.Wireframe = false;
	this->Material.Lighting = false;

	// draw reconstruction points
	this->size = (int)reconstructionPoints->size();
	if(this->Vertices) delete this->Vertices;
	this->Vertices = new irr::video::S3DVertex[size];

	for(int i=0; i<size; i++)
	{
		windage::ReconstructionPoint point = (*reconstructionPoints)[i];
		windage::Vector4 position = point.GetPoint();
		CvScalar color = point.GetColor();
		Vertices[i] = irr::video::S3DVertex(position.x, position.y, position.z, 1, 1, 1, irr::video::SColor(255, color.val[2], color.val[1], color.val[0]), 0, 0);
	}

	// draw camera frame
	int count = (int)calibrationList->size();
	this->cameras.resize(count);
	this->cameraImages.resize(count);
	for(int i=0; i<count; i++)
	{
		windage::Calibration* calibration = (*calibrationList)[i];

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
		upVector *= ((double)this->height/2.0) / upVector.getLength();
		rightVector *= ((double)this->width/2.0) / rightVector.getLength();

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
		this->cameraImages[i].setTexture(0, driver->getTexture((*filenameList)[i].c_str()));
		this->cameraImages[i].Lighting = false;
		this->cameraImages[i].Wireframe = false;
		this->cameraImages[i].FrontfaceCulling = false;
		
	}

	return false;
}

void SceneNode::render()
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