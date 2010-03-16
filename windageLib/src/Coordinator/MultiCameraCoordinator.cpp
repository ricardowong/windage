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

#include "Coordinator/MultiCameraCoordinator.h"
using namespace windage;
using namespace windage::Coordinator;

Vector3 MultiCameraCoordinator::GetTranslation(Calibration* baseCalibration, Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = baseCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.x = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 0, 3);
	fromTranslation.y = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 1, 3);
	fromTranslation.z = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 2, 3);

	windage::Vector3 toTranslation;
	toTranslation.x = CV_MAT_ELEM((*toExtrinsicMatrix), double, 0, 3);
	toTranslation.y = CV_MAT_ELEM((*toExtrinsicMatrix), double, 1, 3);
	toTranslation.z = CV_MAT_ELEM((*toExtrinsicMatrix), double, 2, 3);

	double x = toTranslation.x - fromTranslation.x;
	double y = toTranslation.y - fromTranslation.y;
	double z = toTranslation.z - fromTranslation.z;

	return windage::Vector3(x, y, z);
}

Matrix3 MultiCameraCoordinator::GetRotation(Calibration* baseCalibration, Calibration* toCalibration)
{
	CvMat* toExtrinsicMatrix = toCalibration->GetExtrinsicMatrix();
	CvMat* fromExtrinsicMatrix = baseCalibration->GetExtrinsicMatrix();

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = CV_MAT_ELEM((*fromExtrinsicMatrix), double, y, x);
		}
	}
	
	windage::Matrix3 toRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			toRotation.m[y][x] = CV_MAT_ELEM((*toExtrinsicMatrix), double, y, x);
		}
	}
	
	return fromRotation * toRotation.Transpose();
}

Matrix4 MultiCameraCoordinator::CalculateExtrinsic(Calibration* baseCalibration, Matrix3 toRotation, Vector3 toTranslation)
{
	CvMat* fromExtrinsicMatrix = baseCalibration->GetExtrinsicMatrix();

	windage::Vector3 fromTranslation;
	fromTranslation.v[0] = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 0, 3);
	fromTranslation.v[1] = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 1, 3);
	fromTranslation.v[2] = CV_MAT_ELEM((*fromExtrinsicMatrix), double, 2, 3);

	windage::Matrix3 fromRotation;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			fromRotation.m[y][x] = CV_MAT_ELEM((*fromExtrinsicMatrix), double, y, x);
		}
	}

//	fromTranslation = toRotation * fromTranslation;

	windage::Matrix3 rotation = toRotation * fromRotation;
	windage::Vector3 translation = fromTranslation + toTranslation;

	// set rotatino and translation
	windage::Matrix4 matrix;
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			matrix.m[y][x] = rotation.m[y][x];
		}
		matrix.m[y][3] = translation.v[y];
	}

	matrix.m[3][0] = matrix.m[3][1] = matrix.m[3][2] = 0.0;
	matrix.m[3][3] = 1.0;

	return matrix;
}