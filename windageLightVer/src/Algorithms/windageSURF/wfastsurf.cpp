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

#include "Algorithms/windageSURF/wfastsurf.h"

const int dx1[] = {3, 3, 2, 1, 0, -1, -2, -3};
const int dx2[] = {-3, -3, -2, -1, 0, 1, 2, 3};
const int dy1[] = {0, 1, 2, 3, 3, 3, 2, 1};
const int dy2[] = {0, -1, -2, -3, -3, -3, -2, -1};

// modified FAST SURF descriptor
void wExtractFASTSURF(const IplImage* image, std::vector<windage::FeaturePoint>* keypoints)
{
	const int PATCH_SZ = 25;
	int N = keypoints->size();

	int PATCH[PATCH_SZ+1][PATCH_SZ+1];
	float DX[PATCH_SZ][PATCH_SZ];
	float DY[PATCH_SZ][PATCH_SZ];
	
//	#pragma omp parallel for schedule(dynamic)
    for(int k = 0; k < N; k++ )
    {
		int i, j;
		int x = cvRound((*keypoints)[k].GetPoint().x);
		int y = cvRound((*keypoints)[k].GetPoint().y);

		// calculate rotation
		int dx = 0;
		int dy = 0;
		for(int i=0; i<8; i++)
		{
			int intensity1 = (int)(unsigned char)image->imageData[(y+dy1[i])*image->widthStep + (x+dx1[i])];
			int intensity2 = (int)(unsigned char)image->imageData[(y+dy2[i])*image->widthStep + (x+dx2[i])];
			int difference = intensity1 - intensity2;
			dx += dx1[i] * difference;
			dy += dy1[i] * difference;
		}

        float descriptor_dir = -cvFastArctan( (float)dy, (float)dx );
        descriptor_dir *= (float)(CV_PI/180.0f);
		(*keypoints)[k].SetDir(descriptor_dir);

		// extract descriptors
		float sin_dir = sin(descriptor_dir);
        float cos_dir = cos(descriptor_dir) ;

        // Nearest neighbour version (faster)
        float win_offset = -(float)(PATCH_SZ-1)/2;
        float start_x = x + win_offset*cos_dir + win_offset*sin_dir;
        float start_y = y - win_offset*sin_dir + win_offset*cos_dir;
        for( i=0; i<PATCH_SZ+1; i++, start_x+=sin_dir, start_y+=cos_dir )
        {
            float pixel_x = start_x;
            float pixel_y = start_y;
            for( j=0; j<PATCH_SZ+1; j++, pixel_x+=cos_dir, pixel_y-=sin_dir )
            {
                x = cvRound( pixel_x );
                y = cvRound( pixel_y );
                x = MAX( x, 0 );
                y = MAX( y, 0 );
                x = MIN( x, image->width-1 );
                y = MIN( y, image->height-1 );
				PATCH[i][j] = (int)(unsigned char)image->imageData[y*image->widthStep + x];
             }
        }

        // Calculate gradients in x and y with wavelets of size 2s
        for( i = 0; i < PATCH_SZ; i++ )
		{
            for( j = 0; j < PATCH_SZ; j++ )
            {
				DX[i][j] = (float)(PATCH[i][j+1] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i+1][j]);
				DY[i][j] = (float)(PATCH[i+1][j] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i][j+1]);
            }
		}

        // Construct the descriptor
		// always 36-bin descriptor
		int index = 0;
        for(i = 0; i < 3; i++)
		{
            for(j = 0; j < 3; j++)
            {
                for(y = i*5; y < i*5+5; y++)
                {
                    for(x = j*5; x < j*5+5; x++)
                    {
                        (*keypoints)[k].descriptor[index + 0] += DX[y][x];
						(*keypoints)[k].descriptor[index + 1] += DY[y][x];
                        (*keypoints)[k].descriptor[index + 2] += fabs(DX[y][x]);
						(*keypoints)[k].descriptor[index + 3] += fabs(DY[y][x]);
                    }
                }
                index+=4;
			}
		}
    }
}

