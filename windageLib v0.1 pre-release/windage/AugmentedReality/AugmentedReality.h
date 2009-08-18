#ifndef _AUGMENTED_REALITY_H_
#define _AUGMENTED_REALITY_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include "Tracker/Calibration.h"

namespace windage
{
	class DLLEXPORT AugmentedReality
	{
	protected:
		Calibration* cameraParameter;

		bool isFlip;
		int imageWidth;
		int imageHeight;
		int textureWidth;

		IplImage* textureRepository;
		
		virtual void Release() = 0;

	public:
		AugmentedReality();
		virtual ~AugmentedReality();
		inline void AttatchCameraParameter(Calibration* cameraParameter){this->cameraParameter = cameraParameter;};

		virtual void DrawBackgroundTexture(IplImage* inputImage) = 0;
		virtual void SetProjectionMatrix() = 0;
		virtual void SetModelViewMatrix() = 0;
	};
}

#endif