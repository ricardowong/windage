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

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>

namespace windage
{

	class DLLEXPORT Calibration
	{
	private:
		bool initialized;
		CvMat* intrinsicMatrix;
		CvMat* distortionCoefficients;
		CvMat* extrinsicMatrix;

		void Release();
		
	public:
		Calibration();
		~Calibration();

		inline void SetInitialized(bool state){this->initialized = state;};
		
		inline CvMat* GetIntrinsicMatrix(){return intrinsicMatrix;};
		inline CvMat* GetDistortionCoefficients(){return distortionCoefficients;};
		inline CvMat* GetExtrinsicMatrix(){return extrinsicMatrix;};

		void Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4);
		void SetIntrinsicMatirx(double fx, double fy, double cx, double cy);
		void SetDistortionCoefficients(double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
		void SetExtrinsicMatrix(CvMat* matrix);
		void SetExtrinsicMatrix(float* matrix);
		void SetExtrinsicMatrix(double* matrix);

		void ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector);

		void GetCameraPosition(CvMat* output3vector);
		CvScalar GetCameraPosition();

		// utils
		int ConvertWorld2Camera(CvMat* output4vector, CvMat* input4vector);
		int ConvertCamera2Image(CvMat* output3vector, CvMat* input3vector);
		int ConvertWorld2Image(CvMat* output3vector, CvMat* input4vector);
		CvPoint ConvertWorld2Image(double x, double y, double z);
		int ConvertCamera2World(CvMat* output3vector, CvMat* input3vector);
		int ConvertImage2Camera(CvMat* output3vector, CvMat* input3vector, double z);
		int ConvertImage2World(CvMat* output3vector, CvMat* input3vector, double z);
		CvPoint2D64f ConvertImage2World(double ix, double iy, double wz=0.0);

		void Undistortion(IplImage* input, IplImage* output);
	};

}
#endif