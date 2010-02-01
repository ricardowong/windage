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

#ifndef _TEMPLATE_MINIMIZATION_H_
#define _TEMPLATE_MINIMIZATION_H_

#include <vector>
#include <cv.h>

#include "base.h"
#include "Structures/Matrix.h"
 
namespace windage
{
	namespace Algorithms
	{
		class DLLEXPORT TemplateMinimization
		{
		protected:
			static const int DELTA = 1;
			double PARAMETER_AMPLIFICATION;
			int SAMPLING_STEP;
			Matrix3 homography;

			int width;
			int height;

			IplImage* templateImage;
			IplImage* samplingImage;

			bool isInitialize;

		public:
			TemplateMinimization(int width=150, int height=150)
			{
				this->PARAMETER_AMPLIFICATION = 2.0;
				this->SAMPLING_STEP = 3;

				this->width = width;
				this->height = height;

				homography.m1[0] = 1.0; homography.m1[1] = 0.0; homography.m1[2] = 0.0;
				homography.m1[3] = 0.0; homography.m1[4] = 1.0; homography.m1[5] = 0.0;
				homography.m1[6] = 0.0; homography.m1[7] = 0.0; homography.m1[8] = 1.0;

				// templateImage is gray
				templateImage = cvCreateImage(this->GetTemplateSize(), IPL_DEPTH_8U, 1);
				samplingImage = cvCreateImage(this->GetTemplateSize(), IPL_DEPTH_8U, 1);
				cvZero(samplingImage);

				isInitialize = false;
			}
			virtual ~TemplateMinimization()
			{
				if(templateImage)	cvReleaseImage(&templateImage);
				if(samplingImage)	cvReleaseImage(&samplingImage);
			}

			inline void SetParameterAmplification(double amplification){this->PARAMETER_AMPLIFICATION = amplification;};

			// setup before initialize
			inline void SetSamplingStep(int step){this->SAMPLING_STEP = step;};
			
			inline CvSize GetTemplateSize(){return cvSize(this->width, this->height);};
			inline IplImage* GetTemplateImage(){return this->templateImage;};
			inline IplImage* GetSamplingImage(){return this->samplingImage;};

			virtual Matrix3 GetHomography() = 0;
			virtual void SetInitialHomography(Matrix3 homography) = 0;
			
			virtual bool AttatchTemplateImage(IplImage* image) = 0;
			virtual bool Initialize() = 0;
			virtual double UpdateHomography(IplImage* image, double* delta = NULL) = 0;

			void DrawResult(IplImage* image, windage::Matrix3 homography, CvScalar color, int thickness, int width, int height)
			{
				windage::Vector3 pt1(0.0, 0.0, 1.0);
				windage::Vector3 pt2(width, 0.0, 1.0);
				windage::Vector3 pt3(width, height, 1.0);
				windage::Vector3 pt4(0.0, height, 1.0);

				windage::Vector3 outPoint1 = homography * pt1;
				windage::Vector3 outPoint2 = homography * pt2;
				windage::Vector3 outPoint3 = homography * pt3;
				windage::Vector3 outPoint4 = homography * pt4;

				outPoint1 /= outPoint1.z;
				outPoint2 /= outPoint2.z;
				outPoint3 /= outPoint3.z;
				outPoint4 /= outPoint4.z;

				cvLine(image, cvPoint((int)outPoint1.x, (int)outPoint1.y), cvPoint((int)outPoint2.x, (int)outPoint2.y), color, thickness);
				cvLine(image, cvPoint((int)outPoint2.x, (int)outPoint2.y), cvPoint((int)outPoint3.x, (int)outPoint3.y), color, thickness);		
				cvLine(image, cvPoint((int)outPoint3.x, (int)outPoint3.y), cvPoint((int)outPoint4.x, (int)outPoint4.y), color, thickness);
				cvLine(image, cvPoint((int)outPoint4.x, (int)outPoint4.y), cvPoint((int)outPoint1.x, (int)outPoint1.y), color, thickness);
			};
		};
	}
}

#endif