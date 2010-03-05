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

#include "windageTest.h"
#include "windageTestSample.h"
#include "VectorMatrixTest.h"
#include "CalibrationTest.h"
#include "RotationConverterTest.h"

#include "FeaturePointTest.h"
#include "SURFdetectorTest.h"
#include "SIFTdetectorTest.h"
#include "SIFTGPUdetectorTest.h"
#include "WSURFdetectorTest.h"

#include "KDtreeTest.h"
#include "SpilltreeTest.h"
#include "FLANNtreeTest.h"

#include "OpticalFlowTest.h"

#include "RANSACestimatorTest.h"
#include "ProSACestimatorTest.h"
#include "LMedSestimatorTest.h"
#include "OpenCVRANSACestimatorTest.h"

#include "OutlierCheckerTest.h"

#include "KalmanFilterTest.h"

#include "LMmethodTest.h"

#include "PlanarObjectTrackingTest.h"
#include "MultiplePlanarObjectTrackingTest.h"

#include "StereoReconstructionTest.h"


void main()
{

	windageTestSample testSample;
	VectorMatrixTest testMatrixVector;
	CalibrationTest testCalibration;
	RotationConverterTest testRotationConverter;

	FeaturePointTest testFeaturePoint;
	SURFdetectorTest testSURFdetector;
	SIFTdetectorTest testSIFTdetector;
	SIFTGPUdetectorTest testSIFTGPUdetector;
	WSURFdetectorTest testWSURFdetector;

	KDtreeTest testKDtree;
	SpilltreeTest testSpilltree;
	FLANNtreeTest testFLANNtree;

	OpticalFlowTest testOpticalFlow;

	RANSACestimatorTest testRANSACestimator;
	ProSACestimatorTest testProSACestimator;
	LMedSestimatorTest testLMedSestimator;
	OpenCVRANSACestimatorTest testOpenCVRANSACestimator;

	KalmanFilterTest testKalmanFilter;

	OutlierCheckerTest testOutlierChecker;
	LMmethodTest testLMmethod;

	PlanarObjectTrackingTest testPlanarObjectTracking;
	MultiplePlanarObjectTrackingTest testMultiplePlanarObjectTracking;

	StereoReconstructionTest testStereoReconstruction;

	cvNamedWindow("stop");
	cvWaitKey(10*1000);
	cvDestroyAllWindows();
}
