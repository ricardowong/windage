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

#include "Coordinator/ARForOSG.h"
using namespace windage;
using namespace windage::Coordinator;

void ARForOSG::Release()
{
}

void ARForOSG::Initialize(int width, int height)
{
	this->width = width;
	this->height = height;
}

void ARForOSG::DrawBackgroundTexture(IplImage* inputImage)
{
	// not support
}

void ARForOSG::SetProjectionMatrix()
{
	for(int i=0; i<16; i++)
		this->projectionMatrix.m1[i] = 0.0;

	double fx = this->cameraParameter->GetParameters()[0];
	double fy = this->cameraParameter->GetParameters()[1];
	double cx = this->cameraParameter->GetParameters()[2];
	double cy = this->cameraParameter->GetParameters()[3];
	double dWidth = (double)this->width;
	double dHeight = (double)this->height;

	this->projectionMatrix.m[0][0] =  2.0 * fx / dWidth;
	this->projectionMatrix.m[1][1] =  2.0 * fy / dHeight;
	this->projectionMatrix.m[2][0] = -2.0 * cx / dWidth + 1.0;
	this->projectionMatrix.m[2][1] =  2.0 * cy / dHeight - 1.0;
	this->projectionMatrix.m[2][3] = -1.0;

	double z_far  = 10000.0;
	double z_near = 0.01;
	this->projectionMatrix.m[2][2] = (z_far+z_near)/(z_near-z_far);
	this->projectionMatrix.m[3][2] = -2.0 * z_far * z_near / (z_far-z_near);
}

void ARForOSG::SetModelViewMatrix()
{
	CvMat* _RT = this->cameraParameter->GetExtrinsicMatrix();

	this->modelviewMatrix.m1[0] = cvmGet(_RT, 0, 0);
	this->modelviewMatrix.m1[4] = cvmGet(_RT, 0, 1);
	this->modelviewMatrix.m1[8] = cvmGet(_RT, 0, 2);
	this->modelviewMatrix.m1[12]= cvmGet(_RT, 0, 3);

	this->modelviewMatrix.m1[1] = -cvmGet(_RT, 1, 0);
	this->modelviewMatrix.m1[5] = -cvmGet(_RT, 1, 1);
	this->modelviewMatrix.m1[9] = -cvmGet(_RT, 1, 2);
	this->modelviewMatrix.m1[13]= -cvmGet(_RT, 1, 3);

	this->modelviewMatrix.m1[2] = -cvmGet(_RT, 2, 0);
	this->modelviewMatrix.m1[6] = -cvmGet(_RT, 2, 1);
	this->modelviewMatrix.m1[10]= -cvmGet(_RT, 2, 2);
	this->modelviewMatrix.m1[14]= -cvmGet(_RT, 2, 3);

	this->modelviewMatrix.m1[3] = 0.0;
	this->modelviewMatrix.m1[7] = 0.0;
	this->modelviewMatrix.m1[11]= 0.0;
	this->modelviewMatrix.m1[15]= 1.0;
}

windage::Matrix4 ARForOSG::ConvertModelViewMatrix(windage::Matrix4 extrinsic)
{
	windage::Matrix4 localModelView;

	localModelView.m1[0] = extrinsic.m[0][0];
	localModelView.m1[4] = extrinsic.m[0][1];
	localModelView.m1[8] = extrinsic.m[0][2];
	localModelView.m1[12]= extrinsic.m[0][3];

	localModelView.m1[1] = -extrinsic.m[1][0];
	localModelView.m1[5] = -extrinsic.m[1][1];
	localModelView.m1[9] = -extrinsic.m[1][2];
	localModelView.m1[13]= -extrinsic.m[1][3];

	localModelView.m1[2] = -extrinsic.m[2][0];
	localModelView.m1[6] = -extrinsic.m[2][1];
	localModelView.m1[10]= -extrinsic.m[2][2];
	localModelView.m1[14]= -extrinsic.m[2][3];

	localModelView.m1[3] = 0.0;
	localModelView.m1[7] = 0.0;
	localModelView.m1[11]= 0.0;
	localModelView.m1[15]= 1.0;

	return localModelView;
}

windage::Vector3 ARForOSG::ConvertOSGPosition(windage::Vector3 position)
{
	double y = -position.z;
	double z = -position.y;
	position.y = y;
	position.z = z;
	return position;
}