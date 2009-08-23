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

#include "Tracker.h"
using namespace windage;

Tracker::Tracker()
{
	cameraParameter = NULL;
}

Tracker::~Tracker()
{
	this->Release();
}

void Tracker::Release()
{
	if(cameraParameter) delete cameraParameter;
	cameraParameter = NULL;
}

//void Tracker::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4)
//{
//	this->Release();
//	cameraParameter = new Calibration();
//	cameraParameter->Initialize(fx, fy, cx, cy, d1, d2, d3, d4);
//}

//int Tracker::UpdateCameraPose(IplImage *grayImage)
//{
//	return 0;
//}

void Tracker::DrawInfomation(IplImage *colorImage, double size)
{
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(size, 0.0, 0.0), CV_RGB(255, 0, 0), 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(0.0, size, 0.0), CV_RGB(0, 255, 0), 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0), cameraParameter->ConvertWorld2Image(0.0, 0.0, size), CV_RGB(0, 0, 255), 2);
}