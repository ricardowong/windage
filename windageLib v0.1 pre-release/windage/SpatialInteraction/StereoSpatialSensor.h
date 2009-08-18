#ifndef _STEREO_SPATIAL_SENSOR_H_
#define _STEREO_SPATIAL_SENSOR_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>
#include "SpatialInteraction/SpatialSensor.h"
#include "Tracker/Calibration.h"

namespace windage
{
	class DLLEXPORT StereoSpatialSensor : public SpatialSensor
	{
	private:
		static const int CHANNEL = 1;
		int cameraNumber;
		int kernelSize;
		double activationThreshold;
		std::vector<Calibration*> cameraParameters;
		std::vector<IplImage*> kernelImages;

		void Release();

		inline int GetCameraNumber(){return this->cameraNumber;};
		inline void SetCameraNumber(int cameraNumber){this->cameraNumber = cameraNumber;};
		inline int GetKernelSize(){return this->kernelSize*2;};
		void SetKernelSize(int kernelSize);

	public:
		StereoSpatialSensor();
		~StereoSpatialSensor();

		void Initialize(Vector3 position, double activationThreshold = 20.0, double kernelSize = 10.0, int cameraNumber = 2);

		inline double GetActivationThreshold(){return this->activationThreshold;};
		inline void SetActivationThreshold(double activationThreshold){this->activationThreshold = activationThreshold;};

		void AttatchCameraParameter(int cameraNumber, Calibration* cameraParameters);
		
		bool GenerateKernelImage(std::vector<IplImage*>* images);
		double GetDisparity(std::vector<IplImage*>* images);
		bool CalculateActivation(std::vector<IplImage*>* images);

//		void operator=(const StereoSpatialSensor rhs);
	};
}

#endif