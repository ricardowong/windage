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
 * @file	Calibration.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	It has camera information
 *
 *	- Data : intrinsic matrix, distortion coefficient, extrinsic matrix
 *	- Functions : coordination converters, undistortion, draw camera parameter to image
 */

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <cv.h>

#include "base.h"
#include "Structures/Vector.h"

namespace windage
{
	/**
	 * @defgroup Structures Data Structures
	 * @brief
	 *		data structures classes
	 * @addtogroup Structures
	 * @{
	 */

	/**
	 * @brief	Class for camera parameter
	 * @author	Woonhyuk Baek
	 */
	class DLLEXPORT Calibration
	{
	private:
		double parameter[8];			///< intrinsic parameter (Camera coordinate to Image coordinate)

		CvMat* intrinsicMatrix;			///< intrinsic matrix (Camera coordinate to Image coordinate)
		CvMat* distortionCoefficients;	///< distortion coefficient
		CvMat* extrinsicMatrix;			///< extrinsic matrix (World coordinate to Camera coordinate)

		IplImage* dstMapX;				///< pre-calculated data storage for map-based undistortion method
		IplImage* dstMapY;				///< pre-calculated data storage for map-based undistortion method

		/**
		 * @fn	Release
		 * @brief
		 *		release all data memory
		 * @remark
		 *		it is necessary function to delete data and release memories
		 */
		void Release();
		
	public:
		Calibration()
		{
			intrinsicMatrix = cvCreateMat(3, 3, CV_64FC1);
			distortionCoefficients = cvCreateMat(4, 1, CV_64FC1);
			extrinsicMatrix = cvCreateMat(4, 4, CV_64FC1);

			cvmSetZero(intrinsicMatrix);
			cvmSetZero(distortionCoefficients);
			cvmSetZero(extrinsicMatrix);
			for(int i=0; i<4; i++)
				CV_MAT_ELEM((*extrinsicMatrix), double, i, i) = 1.0;

			dstMapX = NULL;
			dstMapY = NULL;
		}
		~Calibration()
		{
			this->Release();
		}

		/**
		 * @fn	operator=
		 * @brief
		 *		overwriting assignment expression
		 * @remark
		 *		copy all data
		 */
		void operator=(Calibration &oprd)
		{
			this->Initialize(oprd.GetParameters()[0], oprd.GetParameters()[1], oprd.GetParameters()[2], oprd.GetParameters()[3],
							 oprd.GetParameters()[4], oprd.GetParameters()[5], oprd.GetParameters()[6], oprd.GetParameters()[7]);
			this->SetExtrinsicMatrix(oprd.GetExtrinsicMatrix());
		}

		inline double* GetParameters(){return this->parameter;};
		inline CvMat* GetIntrinsicMatrix(){return this->intrinsicMatrix;};
		inline CvMat* GetDistortionCoefficients(){return this->distortionCoefficients;};
		inline CvMat* GetExtrinsicMatrix(){return this->extrinsicMatrix;};

		/**
		 * @fn	Initialize
		 * @brief
		 *		initialize camera paramter (intrinsic parameter)
		 * @sa	SetIntrinsicMatirx
		 * @sa	SetDistortionCoefficients
		 * @remark
		 *		it is necessary function 
		 *		can alternate SetIntrinsicMatirx and SetDistortionCoefficients functions but recommended
		 */
		void Initialize(
						double fx,		///< intrinsic parameter x focal length
						double fy,		///< intrinsic parameter y focal length
						double cx, 		///< intrinsic parameter x principle point
						double cy, 		///< intrinsic parameter y principle point
						double d1 = 0,	///< intrinsic parameter distortion factor1
						double d2 = 0,	///< intrinsic parameter distortion factor2
						double d3 = 0,	///< intrinsic parameter distortion factor3
						double d4 = 0	///< intrinsic parameter distortion factor4
						);

		void SetIntrinsicMatirx(double fx, double fy, double cx, double cy);
		void SetDistortionCoefficients(double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
		void SetExtrinsicMatrix(CvMat* matrix);
		void SetExtrinsicMatrix(float* matrix);
		void SetExtrinsicMatrix(double* matrix);

		void ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector);

		CvScalar GetCameraPosition();
		void SetCameraPosition(CvScalar position);
		CvScalar GetLookAt();
		CvScalar GetUpPoint();
		CvScalar GetRightPoint();
		
		/**
		 * @defgroup CalibrationCoordinateConvertor Coordinate Convertor
		 * @brief
		 *		Coordinate Convertor
		 * @remark
		 *		Coordinate Convert World, Camera, Image coordinate
		 * @addtogroup CalibrationCoordinateConvertor
		 * @{
		 */
		int ConvertWorld2Camera(CvMat* output4vector, CvMat* input4vector);
		windage::Vector4 ConvertWorld2Camerad(windage::Vector4 input);
		int ConvertCamera2Image(CvMat* output3vector, CvMat* input3vector);
		int ConvertWorld2Image(CvMat* output3vector, CvMat* input4vector);
		CvPoint ConvertWorld2Image(double x, double y, double z);
		windage::Vector2 ConvertWorld2Imaged(double x, double y, double z);
		int ConvertCamera2World(CvMat* output3vector, CvMat* input3vector);
		CvScalar ConvertCamera2World(double x, double y, double z);
		int ConvertImage2Camera(CvMat* output3vector, CvMat* input3vector, double z);
		int ConvertImage2World(CvMat* output3vector, CvMat* input3vector, double z);
		CvPoint2D64f ConvertImage2World(double ix, double iy, double wz=0.0);
		/** @} */ // Calibration:CoordinateConvertor

		/**
		 * @fn	InitUndistortionMap
		 * @brief
		 *		initialize remapping data for map-based undistortion method
		 * @remark
		 *		pre-calculate undistortion reampping data using distortino coefficents
		 */
		void InitUndistortionMap(
									int width,	///< input image width size
									int height	///< input image height size
								);
		/**
		 * @fn	Undistortion
		 * @brief
		 *		undistortion method
		 * @remark
		 *		if after initialized undistortion map then undistort input image using pre-calculated data (faster)
		 *		else real-time calculate undisotrtion map
		 */
		void Undistortion(IplImage* input, IplImage* output);

		/**
		 * @fn	DrawInfomation
		 * @brief
		 *		draw camera information to image depend on intrinsic & extrinsic
		 * @remark
		 *		size is a relative value 
		 */
		void DrawInfomation(IplImage* colorImage, double size = 10.0);
	};
	/** @} */ // addtogroup Structures
}
#endif // _CALIBRATION_H_