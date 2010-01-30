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

#ifndef _W_SURF_H_
#define _W_SURF_H_

// modified SURF by windage
/** @cond */

#include <cv.h>
#include <cxmisc.h>

const int SURF_DESCRIPTOR_DIMENSION = 36;	///< Modified SURF Descriptor dimension = 36 (fixed)
const int SURF_DESCRIPTOR_TYPE = CV_32F;

/**
 * @brief
 *		structor of SURF Description
 * @author
 *		windage
 */
typedef struct _SURFDescription
{
	CvPoint2D32f point;							///< extract point
	int size;									///< scale value
	int objectID;
	float dir;
	float descriptor[SURF_DESCRIPTOR_DIMENSION];	///< SURF Descriptor 36-dimension
	float distance;

	struct _SURFDescription()
	{
		size = 0;
		objectID = -1;
	}

	void operator=(struct _SURFDescription oprd)
	{
		point = oprd.point;
		size = oprd.size;
		dir = oprd.dir;
		distance = oprd.distance;
		objectID = oprd.objectID;
		for(int i=0; i<SURF_DESCRIPTOR_DIMENSION; i++)
			descriptor[i] = oprd.descriptor[i];
	}
	float getDistance(struct _SURFDescription oprd)
	{
		float sum = 0;
		for(int i=0; i<SURF_DESCRIPTOR_DIMENSION; i++)
			sum += (float)((descriptor[i] - oprd.descriptor[i]) * (descriptor[i] - oprd.descriptor[i]));
		return sum;
	}

}SURFDesciription;


struct CvSurfHF
{
    int p0, p1, p2, p3;
    float w;
};



float wCalcHaarPattern( const int* origin, const CvSurfHF* f, int n );
static void wResizeHaarPattern( const int src[][5], CvSurfHF* dst, int n, int oldSize, int newSize, int widthStep );
int wInterpolateKeypoint( float N9[3][9], int dx, int dy, int ds, CvSURFPoint *point );

static CvSeq* wFastHessianDetector( const CvMat* sum, const CvMat* mask_sum, CvMemStorage* storage, const CvSURFParams* params );
void getGaussianKernel( CvMat* kernel, int n, double sigma, int ktype );
void wExtractSURF( const CvArr* _img, const CvArr* _mask,
							CvSeq** _keypoints, CvSeq** _descriptors,
							CvMemStorage* storage, CvSURFParams params,
							int useProvidedKeyPts);

#endif