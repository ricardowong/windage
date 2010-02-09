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

#include "Algorithms/EPnPestimator.h"
using namespace windage;
using namespace windage::Algorithms;

#include "Algorithms/epnp/epnp.h"

bool EPnPestimator::Calculate()
{
	if(this->cameraParameter == NULL)
		return false;
	int n = (int)this->referencePoints->size();
	if(n < 4)
		return false;
	if(n != (int)this->scenePoints->size())
		return false;

	double fx = this->cameraParameter->GetParameters()[0];
	double fy = this->cameraParameter->GetParameters()[1];
	double cx = this->cameraParameter->GetParameters()[2];
	double cy = this->cameraParameter->GetParameters()[3];

	// Pose estimation using PnP alogrithm from EPFL
	epnp* _epnp = new epnp;
	_epnp->set_internal_parameters(cx, cy, fx, fy);

	_epnp->set_maximum_number_of_correspondences(n);
	_epnp->reset_correspondences();
	for(int i=0; i<n; i++)
	{
		double _X, _Y, _Z, _u, _v;
		_X = (*this->referencePoints)[i].GetPoint().x;
		_Y = (*this->referencePoints)[i].GetPoint().y;
		_Z = (*this->referencePoints)[i].GetPoint().z;
		_u = (*this->scenePoints)[i].GetPoint().x;
		_v = (*this->scenePoints)[i].GetPoint().y;

		_epnp->add_correspondence(_X, _Y, _Z, _u, _v);
	}

	// compute pose
	double _R[3][3], _t[3];
	double _rerror  = _epnp->compute_pose(_R, _t);
	delete _epnp;

	double extrinsic[16];
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			extrinsic[y*4+x] = _R[y][x];
		}
		extrinsic[y*4+3] = _t[y];
	}
	extrinsic[12] = extrinsic[13] = extrinsic[14] = 0.0;
	extrinsic[15] = 1.0;

	this->cameraParameter->SetExtrinsicMatrix(extrinsic);

	return true;
}