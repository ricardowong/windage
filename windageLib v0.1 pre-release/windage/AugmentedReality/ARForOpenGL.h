#ifndef _AUGMENTED_REALITY_FOR_OPENGL_H_
#define _AUGMENTED_REALITY_FOR_OPENGL_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <gl/glut.h>
#include "AugmentedReality.h"

namespace windage
{
	class DLLEXPORT ARForOpenGL : public AugmentedReality
	{
	private:
		GLuint backgroundTexture;

		void Release();

	public:
		ARForOpenGL();
		~ARForOpenGL();

		void Initialize(int imageWidth, int imageHeight, bool isFlip=false, int textureWidth=512);

		void DrawBackgroundTexture(IplImage* inputImage);
		void SetProjectionMatrix();
		void SetModelViewMatrix();
	};
}

#endif