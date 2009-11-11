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

#include "SpatialInteraction/CubeSensorGroup.h"
using namespace windage;

CubeSensorGroup::CubeSensorGroup()
{
	this->id = -1;
	this->position = Vector3();
	this->rotation = Vector3();

	cellSize = 0;
	cellSpacing = 0;
	arragneCount = 0;
}

CubeSensorGroup::~CubeSensorGroup()
{
}

void CubeSensorGroup::Release()
{
	for(unsigned int i=0; i<this->sensors.size(); i++)
	{
		delete this->sensors[0];
		this->sensors.erase(this->sensors.begin());
	}
	this->sensors.clear();
}

void CubeSensorGroup::Initialize(int id, Vector3 position, int arrangeCount, double arrangeSize)
{
	double cellSize = arrangeSize / (double)(arrangeCount * 2);
	double cellSpacing = cellSize;
	this->Initialize(id, position, arrangeCount, cellSize, cellSpacing);
}

void CubeSensorGroup::Initialize(int id, Vector3 position, int arrangeCount, double cellSize, double cellSpacing)
{
	this->Release();

	this->SetID(id);

	this->SetArrangeCount(arrangeCount);
	this->SetCellSize(cellSize);
	this->SetCellSpacing(cellSpacing);

	double spacing = cellSize + cellSpacing;
	for(int z=-arrangeCount/2; z<=arrangeCount/2; z++)
	{
		for(int y=-arrangeCount/2; y<=arrangeCount/2; y++)
		{
			for(int x=-arrangeCount/2; x<=arrangeCount/2; x++)
			{
				SpatialSensor* tempSensor = new SpatialSensor();
				tempSensor->Initialize(Vector3(spacing*x, spacing*y, spacing*z));

				this->sensors.push_back(tempSensor);
			}
		}
	}

	this->SetPosition(position);
}