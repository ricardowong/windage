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

#include <fstream>

#include "Reconstruction/Utilities/Loader.h"
using namespace windage;
using namespace Reconstruction;

bool Loader::DoLoad(const char* filename)
{
	if(calibrationList == NULL)
		return false;
	if(filenameList == NULL)
		return false;
	if(reconstructionPoints == NULL)
		return false;

	std::ifstream input;
	input.open(filename);
	if(!input.is_open())
		return false;

	const int BUFFER_SIZE = 1024;
	char BUFFER[BUFFER_SIZE];

	// dummay
	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); // # w
	input.getline(BUFFER, BUFFER_SIZE); // # 
	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); //
	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); // # C
	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); //

	// calibration
	int calibrationCount = 0;
	input >> calibrationCount;
	input.getline(BUFFER, BUFFER_SIZE); //
	input.getline(BUFFER, BUFFER_SIZE); // 

	windage::Matrix3 intrinsic;
	windage::Matrix4 extrinsic;

	this->calibrationList->resize(calibrationCount);
	this->filenameList->resize(calibrationCount);
	for(int i=0; i<calibrationCount; i++)
	{
		input.getline(BUFFER, BUFFER_SIZE); // # camera
		input.getline(BUFFER, BUFFER_SIZE); // # intrinsic
		for(int y=0; y<3; y++)
		{
			for(int x=0; x<3; x++)
			{
				input >> intrinsic.m[y][x];
			}
		}

		input.getline(BUFFER, BUFFER_SIZE); // 
		input.getline(BUFFER, BUFFER_SIZE); // 
		input.getline(BUFFER, BUFFER_SIZE); // # extrinsic
		for(int y=0; y<4; y++)
		{
			for(int x=0; x<4; x++)
			{
				input >> extrinsic.m[y][x];
			}
		}

		input.getline(BUFFER, BUFFER_SIZE); // 
		input.getline(BUFFER, BUFFER_SIZE); // 
		
		(*calibrationList)[i] = new windage::Calibration();
		(*calibrationList)[i]->Initialize(intrinsic._11, intrinsic._22, intrinsic._13, intrinsic._23);
		(*calibrationList)[i]->SetExtrinsicMatrix(extrinsic.m1);

		input.getline(BUFFER, BUFFER_SIZE); // # image file
		input.getline(BUFFER, BUFFER_SIZE); // data
		(*filenameList)[i] = std::string(BUFFER);
		
		input.getline(BUFFER, BUFFER_SIZE); // 
	}


	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); // # R
	input.getline(BUFFER, BUFFER_SIZE); // ##
	input.getline(BUFFER, BUFFER_SIZE); //
	
	// reconstruction points
	int reconstructionCount = 0;
	input >> reconstructionCount;
	input.getline(BUFFER, BUFFER_SIZE); //
	input.getline(BUFFER, BUFFER_SIZE); //

	this->reconstructionPoints->resize(reconstructionCount);
	windage::Vector4 point;
	int objectID;
	CvScalar color;

	for(int i=0; i<reconstructionCount; i++)
	{
		if(i % 1000 == 0)
			std::cout << "load reconstruction point : " << i << std::endl;
		windage::ReconstructionPoint reconstructionPoint;

		input.getline(BUFFER, BUFFER_SIZE); // 
		input.getline(BUFFER, BUFFER_SIZE); // # reconstruction
		input.getline(BUFFER, BUFFER_SIZE); //

		// point
		input >> point.x >> point.y >> point.z >> point.w;
		input.getline(BUFFER, BUFFER_SIZE); //
		reconstructionPoint.SetPoint(point);

		input >> objectID;
		input.getline(BUFFER, BUFFER_SIZE); //
		reconstructionPoint.SetObjectID(objectID);

		input >> color.val[0] >> color.val[1] >> color.val[2] >> color.val[3];
		input.getline(BUFFER, BUFFER_SIZE); //
		reconstructionPoint.SetColor(color);

		input.getline(BUFFER, BUFFER_SIZE); // 
		input.getline(BUFFER, BUFFER_SIZE); // # feature point datas

		int featureCount = 0;
		input >> featureCount;
		input.getline(BUFFER, BUFFER_SIZE); // 

		for(int j=0; j<featureCount; j++)
		{
			windage::FeaturePoint featurePoint;
			windage::Vector3 featurePosition;
			int size;
			double dir;
			double distance;

			input.getline(BUFFER, BUFFER_SIZE); // # feature point

			input >> featurePosition.x >> featurePosition.y >> featurePosition.z;
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetPoint(featurePosition);

			input >> objectID;
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetObjectID(objectID);

			input >> color.val[0] >> color.val[1] >> color.val[2] >> color.val[3];
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetColor(color);

			input >> size;
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetSize(size);

			input >> dir;
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetDir(dir);

			input >> distance;
			input.getline(BUFFER, BUFFER_SIZE);
			featurePoint.SetDistance(distance);

			input.getline(BUFFER, BUFFER_SIZE); // # descriptor datas
			int descriptorDimension = 0;
			input >> descriptorDimension;
			input.getline(BUFFER, BUFFER_SIZE); //

			featurePoint.DESCRIPTOR_DIMENSION = descriptorDimension;
			featurePoint.descriptor.resize(descriptorDimension);
			for(int k=0; k<descriptorDimension; k++)
			{
				input >> featurePoint.descriptor[k];
			}
			input.getline(BUFFER, BUFFER_SIZE);

			// add feature point information
			reconstructionPoint.AddFeaturePoint(featurePoint);
		}
		input.getline(BUFFER, BUFFER_SIZE);

		(*this->reconstructionPoints)[i] = reconstructionPoint;
	}
	std::cout << "load reconstruction all point count : " << reconstructionCount << std::endl;

	input.close();

	return true;
}