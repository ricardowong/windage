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

#include "FeatureMatching/CameraPoseCalculator.h"

using namespace windage;
void CameraPoseCalculator::Update(std::vector<Vector2>* imagePoints, std::vector<Vector3>* objectPoints, Calibration* calibration)
{
	CvMat* rotationVector = cvCreateMat(3, 1, CV_64FC1);
	CvMat* translationVector = cvCreateMat(3, 1, CV_64FC1);
	int pointCount = imagePoints->size();

	CvMat* pImagePoints = cvCreateMat(pointCount, 2, CV_64FC1);
	CvMat* pObjectPoints = cvCreateMat(pointCount, 3, CV_64FC1);

	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pImagePoints, i, 0, (*imagePoints)[i].x);
		cvSetReal2D(pImagePoints, i, 1, (*imagePoints)[i].y);
	}
	for(int i=0; i<pointCount; ++i)
	{
		cvSetReal2D(pObjectPoints, i, 0, (*objectPoints)[i].x);
		cvSetReal2D(pObjectPoints, i, 1, (*objectPoints)[i].y);
		cvSetReal2D(pObjectPoints, i, 2, (*objectPoints)[i].z);
	}

	cvFindExtrinsicCameraParams2(pObjectPoints, pImagePoints, calibration->GetIntrinsicMatrix(), calibration->GetDistortionCoefficients(), rotationVector, translationVector);
	calibration->ConvertExtrinsicParameter(rotationVector, translationVector);

	// release matrix
	if(pImagePoints)		cvReleaseMat(&pImagePoints);
	if(pObjectPoints)		cvReleaseMat(&pObjectPoints);
	if(rotationVector)		cvReleaseMat(&rotationVector);
	if(translationVector)	cvReleaseMat(&translationVector);
}


/** borrowed code from bazAR */
void CameraPoseCalculator::DecomposeHomographyToRT(CvMat *intrinsic, CvMat *Homography, CvMat *RT)
{
	int i, j;

	if(Homography != NULL)
	{
		CvMat *invIntrinsic = cvCloneMat(intrinsic);
		cvInv(intrinsic, invIntrinsic);

		// Vectors holding columns of H and R:
		float a_H1[3];
		CvMat  m_H1 = cvMat( 3, 1, CV_32FC1, a_H1 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H1, i, 0, cvmGet( Homography, i, 0 ) );

		float a_H2[3];
		CvMat  m_H2 = cvMat( 3, 1, CV_32FC1, a_H2 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H2, i, 0, cvmGet( Homography, i, 1 ) );

		float a_H3[3];
		CvMat  m_H3 = cvMat( 3, 1, CV_32FC1, a_H3 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H3, i, 0, cvmGet( Homography, i, 2 ) );

		float a_CinvH1[3];
		CvMat  m_CinvH1 = cvMat( 3, 1, CV_32FC1, a_CinvH1 );

		float a_R1[3];
		CvMat  m_R1 = cvMat( 3, 1, CV_32FC1, a_R1 );

		float a_R2[3];
		CvMat  m_R2 = cvMat( 3, 1, CV_32FC1, a_R2 );

		float a_R3[3];
		CvMat  m_R3 = cvMat( 3, 1, CV_32FC1, a_R3 );

		// The rotation matrix:
		float a_R[9];
		CvMat  m_R = cvMat( 3, 3, CV_32FC1, a_R );

		// The translation vector:
		float a_T[3];
		CvMat  m_T = cvMat( 3, 1, CV_32FC1, a_T );

		////////////////////////////////////////////////////////
		// Create norming factor lambda:
		cvGEMM(invIntrinsic, &m_H1, 1, NULL, 0, &m_CinvH1, 0 );

		// Search next orthonormal matrix:
		if( cvNorm( &m_CinvH1, NULL, CV_L2, NULL ) != 0 )
		{
			float lambda = 1.00/cvNorm( &m_CinvH1, NULL, CV_L2, NULL );

			// Create normalized R1 & R2:
			cvGEMM( invIntrinsic, &m_H1, lambda, NULL, 0, &m_R1, 0 );
			cvGEMM( invIntrinsic, &m_H2, lambda, NULL, 0, &m_R2, 0 );

			// Get R3 orthonormal to R1 and R2:
			cvCrossProduct( &m_R1, &m_R2, &m_R3 );

			// Put the rotation column vectors in the rotation matrix:
			for( i = 0; i < 3; i++ )
			{
				cvmSet( &m_R, i, 0,  cvmGet( &m_R1, i, 0 ) );
				cvmSet( &m_R, i, 1,  cvmGet( &m_R2, i, 0 ) );
				cvmSet( &m_R, i, 2,  cvmGet( &m_R3, i, 0 ) );
			}

			// Calculate Translation Vector T (- because of its definition):
			cvGEMM( invIntrinsic, &m_H3, lambda, NULL, 0, &m_T, 0 );

			// Transformation of R into - in Frobenius sense - next orthonormal matrix:
			float a_W[9]; CvMat  m_W  = cvMat( 3, 3, CV_32FC1, a_W  );
			float a_U[9]; CvMat  m_U  = cvMat( 3, 3, CV_32FC1, a_U  );
			float a_Vt[9]; CvMat  m_Vt = cvMat( 3, 3, CV_32FC1, a_Vt );
			cvSVD( &m_R, &m_W, &m_U, &m_Vt, CV_SVD_MODIFY_A | CV_SVD_V_T );
			cvMatMul( &m_U, &m_Vt, &m_R );

			cvReleaseMat(&invIntrinsic);

			cvSetIdentity(RT);
			for(i=0; i<3; i++)
			{
			for(j=0; j<3; j++)
			{
				cvmSet(RT, i, j, cvmGet(&m_R, i, j));
			}
			cvmSet(RT, i, 3, cvmGet(&m_T, i, 0));
			}
		}

	}
}


double CameraPoseCalculator::Update(std::vector<Vector2>* imagePoints, std::vector<Vector2>* referencePoints, Calibration* calibration, bool deleteOutlier)
{
	float homography[9];
	float intrinsicMatrix[9];
	float extrinsicMatrix[12];
	float extrinsicOutMatrix[16];

	std::vector<CvPoint2D32f> pt1;
	std::vector<CvPoint2D32f> pt2;
	for(int i=0; i<imagePoints->size(); i++)
	{
		pt1.push_back(cvPoint2D32f((*referencePoints)[i].x, (*referencePoints)[i].y));
		pt2.push_back(cvPoint2D32f((*imagePoints)[i].x, (*imagePoints)[i].y));
	}

	CvMat* intrinsic = calibration->GetIntrinsicMatrix();
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			intrinsicMatrix[y*3+x] = (float)cvGetReal2D(intrinsic, y, x);
		}
	}

	const double ERROR_BOUND = 5.0;

	float homographyError = 0;
	int n = (int)pt1.size();
	if(n >= 4)
	{
		CvMat _h = cvMat(3, 3, CV_32F, homography);

		CvMat _intrinsic = cvMat(3, 3, CV_32F, intrinsicMatrix);
		CvMat _extrinsic = cvMat(3, 4, CV_32FC1, extrinsicMatrix);

		CvMat _pt1 = cvMat(1, n, CV_32FC2, &(pt1[0]));
		CvMat _pt2 = cvMat(1, n, CV_32FC2, &(pt2[0]));

		homographyError = 0.0;
		if(cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, ERROR_BOUND))
		{

			if(deleteOutlier)
			{
				// calculate homography error & remove outlier
				float difference = 0;
				int count = 0;
				for(unsigned int i=0; i<referencePoints->size(); i++)
				{

					float point[3];
					point[0] = (*referencePoints)[i].x;
					point[1] = (*referencePoints)[i].y;
					point[2] = 1.0;

					float projectionPointX = homography[0] * point[0] + homography[1] * point[1] + homography[2] * point[2];
					float projectionPointY = homography[3] * point[0] + homography[4] * point[1] + homography[5] * point[2];
					float projectionPointZ = homography[6] * point[0] + homography[7] * point[1] + homography[8] * point[2];
					projectionPointX /= projectionPointZ;
					projectionPointY /= projectionPointZ;

					if(abs((*imagePoints)[i].x - projectionPointX) + abs((*imagePoints)[i].y - projectionPointY) <= ERROR_BOUND)
					{
						difference += abs((*imagePoints)[i].x - projectionPointX);
						difference += abs((*imagePoints)[i].y - projectionPointY);
						count++;
					}
					else // outlier
					{
						referencePoints->erase(referencePoints->begin() + count);
						imagePoints->erase(imagePoints->begin() + count);
					}
				}
				homographyError = (difference / 2.) / (float)count;
			}

			DecomposeHomographyToRT(&_intrinsic, &_h, &_extrinsic);

//			memset(extrinsicOutMatrix, 0, sizeof(double)*16);
			for(int y=0; y<3; y++)
			{
				for(int x=0; x<4; x++)
				{
					extrinsicOutMatrix[y*4 + x] = extrinsicMatrix[y*4 + x];
				}
			}
			extrinsicOutMatrix[3*4 + 0] = extrinsicOutMatrix[3*4 + 1] = extrinsicOutMatrix[3*4 + 2] = 0;
			extrinsicOutMatrix[3*4 + 3] = 1;

			calibration->SetExtrinsicMatrix(extrinsicOutMatrix);
		}
		else
		{
			homographyError = -1.0;
		}
	}
	else
	{
		homographyError = -1.0;
	}

	return homographyError;
}