#include "AugmentedReality.h"
using namespace windage;

AugmentedReality::AugmentedReality()
{
	cameraParameter = NULL;

	isFlip = false;
	imageWidth = 0;
	imageHeight = 0;
	textureWidth = 0;

	textureRepository = NULL;
}
AugmentedReality::~AugmentedReality()
{

}
