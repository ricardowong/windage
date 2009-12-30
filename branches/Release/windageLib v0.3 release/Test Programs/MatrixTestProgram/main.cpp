
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

#include <iostream>
#include <windage.h>

void main()
{
	double dx = 15.0 * CV_PI/180.0;
	double dy = 172.0* CV_PI/180.0;
	double dz = (0.0 + 112.0) * CV_PI/180.0;

	windage::Vector3 euler = windage::Vector3(dx, dy, dz);
	windage::Vector4 quaternion = windage::Quaternion::EulerToQuaternion(euler);
	windage::Vector3 confirmEuler = windage::Quaternion::QuaternionToEuler(quaternion);
//	confirmEuler = windage::Quaternion::QuaternionToEuler(windage::Quaternion::EulerToQuaternion(confirmEuler));

	std::cout << "EulerToQuaternion and QuaternionToEuler Error : " << euler.getDistance(confirmEuler) << std::endl;


	windage::Matrix3 dcm = windage::Quaternion::QuaternionToDcm(quaternion);
	windage::Vector4 confirmQuat = windage::Quaternion::DcmToQuaternion(dcm);

	std::cout << "QuaternionToDcm and DcmToQuaternion Error : " << quaternion.getDistance(confirmQuat) << std::endl;


	dcm = windage::Quaternion::EulerToDcm(euler);
	confirmEuler = windage::Quaternion::DcmToEuler(dcm);

	std::cout << "EulerToDcm and DcmToEuler Error : " << euler.getDistance(confirmEuler) << std::endl;



}
