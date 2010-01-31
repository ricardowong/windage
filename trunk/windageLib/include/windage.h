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

// Structures
#include "Structures/Vector.h"
#include "Structures/Matrix.h"

#include "Structures/FeaturePoint.h"
#include "Structures/SURFpoint.h"
#include "Structures/SIFTpoint.h"
#include "Structures/WSURFpoint.h"

#include "Structures/Calibration.h"

// Algorithms
// detector
#include "Algorithms/FeatureDetector.h"
#include "Algorithms/SURFdetector.h"
#include "Algorithms/SIFTdetector.h"
#include "Algorithms/WSURFdetector.h"

// matcher
#include "Algorithms/SearchTree.h"
#include "Algorithms/KDtree.h"
#include "Algorithms/Spilltree.h"
#include "Algorithms/FLANNtree.h"

// tracker
#include "Algorithms/OpticalFlow.h"

// estimator
#include "Algorithms/PoseEstimator.h"
#include "Algorithms/HomographyEstimator.h"
#include "Algorithms/RANSACestimator.h"
#include "Algorithms/LMeDSestimator.h"
#include "Algorithms/EPnPEstimator.h"

#include "Algorithms/OutlierChecker.h"

// Utilities
#include "Utilities/Utils.h"
#include "Utilities/Logger.h"

// Frameworks
#include "Frameworks/ObjectTracking.h"
