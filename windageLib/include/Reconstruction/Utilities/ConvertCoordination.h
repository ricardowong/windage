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

/**
 * @file	ConvertCoordination.h
 * @author	Woonhyuk Baek
 * @version 1.0
 * @date	2010.03.01
 * @brief	It is log class for logging at reconstruction informations
 */

#ifndef _CONVERT_COORDINATION_H_
#define _CONVERT_COORDINATION_H_

namespace windage
{
	namespace Reconstruction
	{
		class ConvertCoordination
		{
		private:
			std::vector<windage::ReconstructionPoint>* reconstructionPoints;

		public:
			ConvertCoordination()
			{
				reconstructionPoints = NULL;
			}
			~ConvertCoordination()
			{
			}

			inline void AttatchReconstructionPoint(std::vector<windage::ReconstructionPoint>* reconstructionPoints){this->reconstructionPoints = reconstructionPoints;}

			bool ConvertRotation(windage::Matrix3 rotation)
			{
				if(reconstructionPoints == NULL)
					return false;

				windage::Matrix4 extrinsic;
				for(int y=0; y<3; y++)
				{
					for(int x=0; x<3; x++)
					{
						extrinsic.m[y][x] = rotation.m[y][x];
					}
				}
				extrinsic.m[0][3] = extrinsic.m[1][3] = extrinsic.m[2][3] = 0.0;
				extrinsic.m[3][0] = extrinsic.m[3][1] = extrinsic.m[3][2] = 0.0;
				extrinsic.m[3][3] = 1.0;

				bool result = Convert(extrinsic);
				return result;
			}

			bool ConvertTranslation(windage::Vector3 translation)
			{
				if(reconstructionPoints == NULL)
					return false;

				windage::Matrix4 extrinsic;
				for(int y=0; y<3; y++)
				{
					for(int x=0; x<3; x++)
					{
						if(x == y)
							extrinsic.m[y][x] = 1.0;
						else
							extrinsic.m[y][x] = 0.0;
					}
				}
				extrinsic.m[0][3] = translation.x;
				extrinsic.m[1][3] = translation.y;
				extrinsic.m[2][3] = translation.z;
				extrinsic.m[3][0] = extrinsic.m[3][1] = extrinsic.m[3][2] = 0.0;
				extrinsic.m[3][3] = 1.0;

				bool result = Convert(extrinsic);
				return result;
			}

			bool Convert(windage::Matrix4 extrinsic)
			{
				if(reconstructionPoints == NULL)
					return false;

				for(unsigned int i=0; i<this->reconstructionPoints->size(); i++)
				{
					windage::Vector4 point = (*reconstructionPoints)[i].GetPoint();
					windage::Vector4 cvtPoint = extrinsic * point;
					(*reconstructionPoints)[i].SetPoint(cvtPoint);
				}

				return true;
			}
		};
	}
}

#endif //_CONVERT_COORDINATION_H_