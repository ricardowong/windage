#include "SpatialInteraction/StereoSpatialSensor.h"
using namespace windage;


StereoSpatialSensor::StereoSpatialSensor()
{
	cameraNumber = 0;
	kernelSize = 0;
	activationThreshold = 0;

}

StereoSpatialSensor::~StereoSpatialSensor()
{
}

void StereoSpatialSensor::Release()
{
	for(int i=0; i<kernelImages.size(); i++)
	{
		cvReleaseImage(&kernelImages[i]);
		kernelImages[i] = NULL;
	}
	kernelImages.clear();
}

void StereoSpatialSensor::Initialize(Vector3 position, double activationThreshold, double kernelSize, int cameraNumber)
{
	this->Release();

	this->SetPosition(position);

	this->SetActivationThreshold(activationThreshold);
	this->SetCameraNumber(cameraNumber);
	
	this->SetKernelSize(kernelSize);
}

void StereoSpatialSensor::SetKernelSize(int kernelSize)
{
	this->kernelSize = kernelSize/2;
	
	for(int i=0; i<cameraNumber; i++)
	{
		IplImage* temp = cvCreateImage(cvSize(kernelSize, kernelSize), IPL_DEPTH_8U, CHANNEL);
		this->kernelImages.push_back(temp);
	}
	this->cameraParameters.resize(cameraNumber);
}

void StereoSpatialSensor::AttatchCameraParameter(int cameraNumber, Calibration* cameraParameters)
{
	if(cameraNumber < this->cameraParameters.size())
	{
		this->cameraParameters[cameraNumber] = cameraParameters;
	}
}

bool StereoSpatialSensor::GenerateKernelImage(std::vector<IplImage*>* images)
{
	if((*images).size() == this->kernelImages.size() && this->kernelImages.size() == (unsigned int)this->cameraNumber)
	{
		int width = (*images)[0]->width;
		int height = (*images)[0]->height;

		for(int i=0; i<this->cameraNumber; i++)
		{
			CvPoint imageCoordinate = cameraParameters[i]->ConvertWorld2Image(this->position.x, this->position.y, this->position.z);

			cvZero(this->kernelImages[i]);
			if( this->kernelSize <= imageCoordinate.x && imageCoordinate.x < width-this->kernelSize && 
				this->kernelSize <= imageCoordinate.y && imageCoordinate.y < height-this->kernelSize)
			{
				cvSetImageROI((*images)[i], cvRect(imageCoordinate.x - this->kernelSize, imageCoordinate.y - this->kernelSize, this->kernelSize*2, this->kernelSize*2));
				cvCopy((*images)[i], this->kernelImages[i]);
				cvResetImageROI((*images)[i]);
			}
		}

		return true;
	}
	else
	{
		return false;
	}
}

double StereoSpatialSensor::GetDisparity(std::vector<IplImage*>* images)
{
	double disparity = 0.0;

	CvScalar baseColor;
	CvScalar compareColor;
	for(int i=1; i<this->kernelImages.size(); i++)
	{
		double tempDisparity = 0.0;
		for(int y=0; y<kernelSize*2; y++)
		{
			for(int x=0; x<kernelSize*2; x++)
			{
				baseColor = cvGet2D(this->kernelImages[0], y, x);
				compareColor = cvGet2D(this->kernelImages[i], y, x);

				tempDisparity += abs(baseColor.val[0] - compareColor.val[0]) + 
								 abs(baseColor.val[1] - compareColor.val[1]) + 
								 abs(baseColor.val[2] - compareColor.val[2]) + 
								 abs(baseColor.val[3] - compareColor.val[3]);
			}
		}
		tempDisparity /= (double)((kernelSize*2) * (kernelSize*2));

		disparity += tempDisparity;
	}

	disparity /= (double)(this->kernelImages.size()-1);
	return disparity;
}

bool StereoSpatialSensor::CalculateActivation(std::vector<IplImage*>* images)
{
	this->SetActivation(false);
	if(this->GenerateKernelImage(images))
	{
		double diaparity = this->GetDisparity(images);
		if(diaparity < this->GetActivationThreshold())
		{
			this->SetActivation(true);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
