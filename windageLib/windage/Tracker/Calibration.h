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

		void ConvertExtrinsicParameter(CvMat* rotationVector, CvMat* translationVector);

		void GetCameraPosition(CvMat* output3vector);

		// utils
		int ConvertWorld2Camera(CvMat* output4vector, CvMat* input4vector);
		int ConvertCamera2Image(CvMat* output3vector, CvMat* input3vector);
		int ConvertWorld2Image(CvMat* output3vector, CvMat* input4vector);
		CvPoint ConvertWorld2Image(double x, double y, double z);
		int ConvertCamera2World(CvMat* output3vector, CvMat* input3vector);
		int ConvertImage2Camera(CvMat* output3vector, CvMat* input3vector, double z);
		int ConvertImage2World(CvMat* output3vector, CvMat* input3vector, double z);
		CvPoint2D64f ConvertImage2World(double ix, double iy, double wz=0.0);
	};

}
#endif