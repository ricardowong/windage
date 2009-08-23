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