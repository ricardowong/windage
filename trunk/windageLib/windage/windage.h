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

// header files for tracking
#include "Tracker/Calibration.h"
#include "Tracker/Tracker.h"
#include "Tracker/ChessboardTracker.h"
#include "Tracker/ModifiedSURFTracker.h"

// header files for augmented reality
#include "AugmentedReality/AugmentedReality.h"
#include "AugmentedReality/ARForOpenGL.h"

// header files for 3d reconstruction
#include "Reconstructor/Reconstructor.h"

// header files for spatial interaction
#include "SpatialInteraction/SpatialSensor.h"
#include "SpatialInteraction/SensorGroup.h"
#include "SpatialInteraction/CubeSensorGroup.h"
#include "SpatialInteraction/SensorDetector.h"
#include "SpatialInteraction/StereoSensorDetector.h"
#include "SpatialInteraction/StereoSURFDetector.h"

// utilities header file
#include "Utils/Logger.h"
#include "Utils/wVector.h"
#include "Utils/wMatrix.h"
#include "Utils/Utils.h"

// for Doxygen Mainpage Documentation
/**
 * @mainpage windage Library
 * @section developer Developer
 *		- windage
 *			- windage@live.com
 * @section info Library Information
 *		- Source code is available under the GNU General Public License.
 *		- In short, if you distribute a software that uses windageLib-demo, you have to distribute it under GPL with the source code.
 *		- Another option is to contact us to purchase a commercial license.
 * @section advenced Functions
 *		-# Real-time Natural Feature Tracking 
 *			- <img src="http://windage.googlecode.com/svn/trunk/Document/images/Demo_ModifiedSURF.png"/>
 *		-# Muli-method Tracking
 *			- <img src="http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultimethodTracking.png"/>
 *		-# Augmented Reality 
 *			- <img src="http://windage.googlecode.com/svn/trunk/Document/images/Demo_AugmentedReality.png"/>
 *		-# Stereo based 3D Reconstruction 
 *			- <img src="http://windage.googlecode.com/svn/trunk/Document/images/Demo_StereoReconstruction.png"/>
 *		-# Stereo based Spatial Sensors 
 *			- <img src="http://windage.googlecode.com/svn/trunk/Document/images/Demo_SpatialSensor.png"/>
 * @section advenced For further information please contact 
 *		- Woonhyuk Baek
 *			- <windage@live.com>
 *			-
 *			- GIST U-VR Lab.
 *			- Department of Information and Communication
 *			- Gwangju Institute of Science and Technology
 *			- 1, Oryong-dong, Buk-gu, Gwangju
 *			- South Korea
 */