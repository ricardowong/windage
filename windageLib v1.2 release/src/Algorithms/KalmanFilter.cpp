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

#include "Algorithms/KalmanFilter.h"
using namespace windage;
using namespace windage::Algorithms;

void KalmanFilter::Initialize()
{
	const float A[] = { 1, 0, 0, 1, 0, 0,
						0, 1, 0, 0, 1, 0,
						0, 0, 1, 0, 0, 1,
						0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 1, 0,
						0, 0, 0, 0, 0, 1,};

	memcpy( this->kalman->transition_matrix->data.fl, A, sizeof(A));
	cvSetIdentity( this->kalman->measurement_matrix, cvRealScalar(1) );
	cvSetIdentity( this->kalman->process_noise_cov, cvRealScalar(1e-5) );
	cvSetIdentity( this->kalman->measurement_noise_cov, cvRealScalar(1e-1) );
	cvSetIdentity( this->kalman->error_cov_post, cvRealScalar(1));
}

windage::Vector3 KalmanFilter::Predict()
{
	windage::Vector3 prediction;

	const CvMat* predictionMat = cvKalmanPredict( this->kalman, 0 );

	prediction.x = (double)predictionMat->data.fl[0];
	prediction.y = (double)predictionMat->data.fl[1];
	prediction.z = (double)predictionMat->data.fl[2];

	return prediction;
}

bool KalmanFilter::Correct(windage::Vector3 T)
{
	measurement->data.fl[0] = (float)T.x;
	measurement->data.fl[1] = (float)T.y;
	measurement->data.fl[2] = (float)T.z;

	cvKalmanCorrect( kalman, measurement );
	return true;
}