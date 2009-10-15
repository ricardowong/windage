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

#ifndef _FIND_PROSAC_HOMOGRAPY_H_
#define _FIND_PROSAC_HOMOGRAPY_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <vector>
#include <cv.h>

namespace windage
{

	typedef struct _MatchedPoint
	{
		CvPoint2D32f pointScene;
		CvPoint2D32f pointReference;
		float distance;
		bool isInlier;

		struct _MatchedPoint(CvPoint2D32f pointScene, CvPoint2D32f pointReference, float distance = 0.0)
		{
			this->pointScene = pointScene;
			this->pointReference = pointReference;

			this->distance = distance;
			isInlier = false;
		}

		void operator=(struct _MatchedPoint oprd)
		{
			this->pointScene = oprd.pointScene;
			this->pointReference = oprd.pointReference;
			this->distance = oprd.distance;
			this->isInlier = oprd.isInlier;
		}

		bool operator==(struct _MatchedPoint oprd)
		{
			return this->distance == oprd.distance;
		}
		bool operator<(struct _MatchedPoint oprd)
		{
			return this->distance < oprd.distance;
		}
	}MatchedPoint;

	typedef struct _CompareDistanceLess
	{
		bool operator()(const struct _MatchedPoint& p, const struct _MatchedPoint& q) const
		{
			return p.distance < q.distance;
		}
	}CompareDistanceLess;


	class DLLEXPORT FindPROSACHomography
	{
	private:
		int maxIteration;
		float terminationRatio;
		float reprojectionThreshold;
		float homography[9];

		std::vector<MatchedPoint>* matchedPoints;

	public:
		FindPROSACHomography()
		{
			this->maxIteration = 2000;
			this->terminationRatio = 0.7f;
			this->reprojectionThreshold = 5.0f;
			for(int i=0; i<9; i++)
				homography[i] = 0.0f;

			this->matchedPoints = NULL;
		}
		~FindPROSACHomography()
		{

		}

		inline void SetMaxIteration(int iteration=1000){this->maxIteration = iteration;};
		inline void SetTerminationRatio(float ratio=0.7f){this->terminationRatio = ratio;};
		inline void SetReprojectionThreshold(float reprojectionThreshold=5.0f){this->reprojectionThreshold = reprojectionThreshold;};

		inline void AttatchMatchedPoints(std::vector<MatchedPoint>* matchedPoints){this->matchedPoints = matchedPoints;};
		inline float* GetHomography(){return this->homography;};

		bool Calculate();
	};
}

#endif