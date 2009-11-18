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

#ifndef _STEREO_SENSOR_DETECTOR_H_
#define _STEREO_SENSOR_DETECTOR_H_

#include <vector>
#include <cv.h>

#include "base.h"
#include "SpatialInteraction/SensorDetector.h"
#include "SpatialInteraction/SpatialSensor.h"
#include "Tracker/Calibration.h"

namespace windage
{
	/**
	 * @brief
	 *		Class for Spatial Sensor Detector using stereo image
	 * @author
	 *		windage
	 */
	class DLLEXPORT StereoSensorDetector : public SensorDetector
	{
	private:
		static const int CHANNEL = 1;	///< COLOR CHANNEL (fixed)
		int cameraNumber;				///< number of camera
		int kernelSize;					///< compaired image kernel size
		double activationThreshold;		///< activation threshold
		std::vector<Calibration*> cameraParameters;	///< attatched camera parameter only read (not-update)
		std::vector<IplImage*> kernelImages;		///< kernel image storage

		void Release();

		inline int GetCameraNumber(){return this->cameraNumber;};
		inline void SetCameraNumber(int cameraNumber){this->cameraNumber = cameraNumber;};
		inline int GetKernelSize(){return this->kernelSize*2;};
		void SetKernelSize(int kernelSize);

	public:
		StereoSensorDetector();
		~StereoSensorDetector();

		/**
		 * @brief
		 *		Initialize stereo spatial sensor detector
		 * @remark
		 *		Initialize stereo spatial sensor detector, set activation threshold and kernel size and camera number
		 */
		void Initialize(double activationThreshold = 20.0, double kernelSize = 10.0, int cameraNumber = 2);

		inline double GetActivationThreshold(){return this->activationThreshold;};
		inline void SetActivationThreshold(double activationThreshold){this->activationThreshold = activationThreshold;};

		/**
		 * @brief
		 *		Attatch Camera Parameter
		 * @remark
		 *		attatch camera paramter only read and update from outside
		 */
		void AttatchCameraParameter(
									int cameraNumber,				///< attatching camera parameter position
									Calibration* cameraParameters	///< attatching camera parameter
									);
		/**
		 * @brief
		 *		Generate Kernel Image
		 * @remark
		 *		generate kernel image depend on spatial sensor position and camera parameter
		 */
		bool GenerateKernelImage(
									std::vector<IplImage*>* images,	///< input image list
									SpatialSensor* sensor			///< spatial sensor sensor
								);
		/**
		 * @brief
		 *		Get Disparity
		 * @remark
		 *		compaire kernel images from spatial sensor position
		 */
		double GetDisparity(
							std::vector<IplImage*>* images,	///< input image list
							SpatialSensor* sensor			///< spatial sensor sensor
							);

		/**
		 * @brief
		 *		Calculate Activation
		 * @remark
		 *		calculate activation from attatched all spatial sensors
		 */
		void CalculateActivation(std::vector<IplImage*>* images);

//		void operator=(const StereoSpatialSensor rhs);
	};
}

#endif