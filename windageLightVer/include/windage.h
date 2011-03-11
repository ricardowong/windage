/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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

/**
 * @mainpage windageLib : Visual Object Recognition & Tracking Library
 * @section INTRO
 * - windageLib : Visual Object Recognition & Tracking Library
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.12
 */

// Structures
#include "Structures/Vector.h"
#include "Structures/Matrix.h"

#include "Structures/FeaturePoint.h"
#include "Structures/WSURFpoint.h"

#include "Structures/Calibration.h"

// Algorithms
// detector
#include "Algorithms/FeatureDetector.h"
#include "Algorithms/OpenSURFdetector.h"
#include "Algorithms/WSURFdetector.h"

// matcher
#include "Algorithms/SearchTree.h"
#include "Algorithms/KDtree.h"
#include "Algorithms/Spilltree.h"
#include "Algorithms/FLANNtree.h"

#include "Algorithms/KDforest.h"

// tracker
#include "Algorithms/OpticalFlow.h"

// estimator
#include "Algorithms/PoseEstimator.h"
#include "Algorithms/HomographyEstimator.h"
#include "Algorithms/RANSACestimator.h"
#include "Algorithms/ProSACestimator.h"
#include "Algorithms/LMedSestimator.h"
#include "Algorithms/EPnPestimator.h"
#include "Algorithms/EPnPRANSACestimator.h"
#include "Algorithms/OpenCVRANSACestimator.h"

// refiner
#include "Algorithms/LMmethod.h"
#include "Algorithms/PoseRefiner.h"
#include "Algorithms/PoseLMmethod.h"

#include "Algorithms/OutlierChecker.h"

// marker based
#include "Algorithms/ChessboardDetector.h"

#include "Algorithms/KalmanFilter.h"

// Utilities
#include "Utilities/Utils.h"
#include "Utilities/Logger.h"

// Frameworks
#include "Frameworks/PlanarObjectTracking.h"

// Coordinator
#include "Coordinator/RotationConverter.h"

#include "Coordinator/AugmentedReality.h"

#include "Coordinator/MultiCameraCoordinator.h"
#include "Coordinator/MultiMarkerCoordinator.h"
