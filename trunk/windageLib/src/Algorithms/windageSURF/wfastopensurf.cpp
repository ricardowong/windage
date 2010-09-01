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

#include "Algorithms/windageSURF/wfastopensurf.h"

inline int fRound(float flt)
{
  return (int) floor(flt+0.5f);
}

const float pi = 3.14159f;

const float gauss25 [5][5] = {
  0.01706954162243,0.01144205592615,0.00653580605408,0.00318131834134,0.00131955648461,
  0.01342737701584,0.00900063997939,0.00514124713667,0.00250251364222,0.00103799989504,
  0.00900063997939,0.00603330940534,0.00344628101733,0.00167748505986,0.00069579213743,
  0.00514124713667,0.00344628101733,0.00196854695367,0.00095819467066,0.00039744277546,
  0.00250251364222,0.00167748505986,0.00095819467066,0.00046640341759,0.00019345616757,
};

const double gauss33 [11][11] = {
  0.014614763,0.013958917,0.012162744,0.00966788,0.00701053,0.004637568,0.002798657,0.001540738,0.000773799,0.000354525,0.000148179,
  0.013958917,0.013332502,0.011616933,0.009234028,0.006695928,0.004429455,0.002673066,0.001471597,0.000739074,0.000338616,0.000141529,
  0.012162744,0.011616933,0.010122116,0.008045833,0.005834325,0.003859491,0.002329107,0.001282238,0.000643973,0.000295044,0.000123318,
  0.00966788,0.009234028,0.008045833,0.006395444,0.004637568,0.003067819,0.001851353,0.001019221,0.000511879,0.000234524,9.80224E-05,
  0.00701053,0.006695928,0.005834325,0.004637568,0.003362869,0.002224587,0.001342483,0.000739074,0.000371182,0.000170062,7.10796E-05,
  0.004637568,0.004429455,0.003859491,0.003067819,0.002224587,0.001471597,0.000888072,0.000488908,0.000245542,0.000112498,4.70202E-05,
  0.002798657,0.002673066,0.002329107,0.001851353,0.001342483,0.000888072,0.000535929,0.000295044,0.000148179,6.78899E-05,2.83755E-05,
  0.001540738,0.001471597,0.001282238,0.001019221,0.000739074,0.000488908,0.000295044,0.00016243,8.15765E-05,3.73753E-05,1.56215E-05,
  0.000773799,0.000739074,0.000643973,0.000511879,0.000371182,0.000245542,0.000148179,8.15765E-05,4.09698E-05,1.87708E-05,7.84553E-06,
  0.000354525,0.000338616,0.000295044,0.000234524,0.000170062,0.000112498,6.78899E-05,3.73753E-05,1.87708E-05,8.60008E-06,3.59452E-06,
  0.000148179,0.000141529,0.000123318,9.80224E-05,7.10796E-05,4.70202E-05,2.83755E-05,1.56215E-05,7.84553E-06,3.59452E-06,1.50238E-06
};

const int dx1[] = {3, 3, 2, 1, 0, -1, -2, -3};
const int dx2[] = {-3, -3, -2, -1, 0, 1, 2, 3};
const int dy1[] = {0, 1, 2, 3, 3, 3, 2, 1};
const int dy2[] = {0, -1, -2, -3, -3, -3, -2, -1};

float getOrientation(IplImage* image, windage::FeaturePoint* point)
{
	int x = cvRound(point->GetPoint().x);
	int y = cvRound(point->GetPoint().y);

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

    float descriptor_dir = cvFastArctan( (float)dy, (float)dx );
    descriptor_dir *= (float)(CV_PI/180.0f);

	// assign orientation of the dominant response vector
	point->SetDir((double)descriptor_dir);
	return descriptor_dir;
}

void getDescriptor(IplImage* image, windage::FeaturePoint* point)
{
  int x = fRound(point->GetPoint().x);
  int y = fRound(point->GetPoint().y);  

  double co = cos(point->GetDir());
  double si = sin(point->GetDir());

  int xs, ys, xb, yb, dx1, dy1, dx2, dy2;
  float dx, dy, dxp, dyp, dxm, dym;
  float sum = 0;

  int idx = 0;
  for(dy1=-5; dy1<=5; dy1+=5)
  {
	  for(dx1=-5; dx1<=5; dx1+=5)
	  {
		  dxp = dyp = dxm = dym = 0;

		  xb = fRound(x + (co*(dx1-2) - si*(dy1-2)));
		  yb = fRound(y + (si*(dx1-2) + co*(dy1-2)));
		  xb = std::min(image->width-1, xb);  xb = max(0, xb);
		  yb = std::min(image->height-1, yb); yb = max(0, yb);
		  for(dy2=-1; dy2<=3; dy2++)
		  {
			  for(dx2=-1; dx2<=3; dx2++)
			  {
				  xs = fRound(x + (co*(dx1+dx2) - si*(dy1+dy2)));
				  ys = fRound(y + (si*(dx1+dx2) + co*(dy1+dy2)));
				  xs = std::min(image->width-1, xs);  xs = max(0, xs);
				  ys = std::min(image->height-1, ys); ys = max(0, ys);
				  
				  dx = gauss25[dy2+1][dx2+1] * (float)((int)CV_IMAGE_ELEM(image, unsigned int, ys, xs)-(int)CV_IMAGE_ELEM(image, unsigned int, ys, xb));
				  dy = gauss25[dy2+1][dx2+1] * (float)((int)CV_IMAGE_ELEM(image, unsigned int, ys, xs)-(int)CV_IMAGE_ELEM(image, unsigned int, yb, xs));

				  xb = xs;
				  yb = ys;

				  if(dx < 0) dxm += dx;
				  else		 dxp += dx;
				  if(dy < 0) dym += dy;
				  else		 dyp += dy;
			  }
		  }

		  point->descriptor[idx+0] = dxp;
		  point->descriptor[idx+1] = dyp;
		  point->descriptor[idx+2] = dxm;
		  point->descriptor[idx+3] = dym;
		  sum += dxp + dyp + dxm + dym;
		  idx += 4;
	  }
  }

  for(int i=0; i<point->DESCRIPTOR_DIMENSION; i++)
	  point->descriptor[i] /= sum;
}


void getDescriptor2(IplImage* image, windage::FeaturePoint* point)
{
	int x = fRound(point->GetPoint().x);
	int y = fRound(point->GetPoint().y);  

	const unsigned int PATCH_SZ = 7;
	unsigned int index;
	unsigned int i, j;
	int dx, dy;
	unsigned int intensity1, intensity2;

	float descriptor_dir, sin_dir, cos_dir;
	float win_offset, start_x, start_y, pixel_x, pixel_y;
	int DX[PATCH_SZ][PATCH_SZ];
	int DY[PATCH_SZ][PATCH_SZ];
	unsigned int PATCH[PATCH_SZ+1][PATCH_SZ+1];
	unsigned int k;
	
	descriptor_dir = (float)point->GetDir();
	
	// extract descriptors
	sin_dir = (float)sin((double)descriptor_dir);
	cos_dir = (float)cos((double)descriptor_dir) ;

	// Nearest neighbour version (faster)
	win_offset = -(float)(PATCH_SZ-1)/2.0f;
	start_x = x + win_offset*cos_dir + win_offset*sin_dir;
	start_y = y - win_offset*sin_dir + win_offset*cos_dir;
	for(i=0; i<PATCH_SZ+1; i++)
	{
		pixel_x = start_x;
		pixel_y = start_y;
		for(j=0; j<PATCH_SZ+1; j++)
		{
			x = unsigned int( pixel_x );
			y = unsigned int( pixel_y );
			x = max( x, 0 );
			y = max( y, 0 );
			x = std::min( x, image->width-1 );
			y = std::min( y, image->height-1 );
			PATCH[i][j] = CV_IMAGE_ELEM(image, unsigned int, y, x);

			pixel_x+=cos_dir;
			pixel_y-=sin_dir;
		}

		start_x+=sin_dir;
		start_y+=cos_dir;
	}

	// Calculate gradients in x and y with wavelets of size 2s
	for(i=0; i<PATCH_SZ; i++)
	{
		for(j=0; j<PATCH_SZ; j++)
		{
			DX[i][j] = (PATCH[i][j+1] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i+1][j]);
			DY[i][j] = (PATCH[i+1][j] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i][j+1]);
		}
	}

	// Construct the descriptor
	

	// always 36-bin descriptor
	index = 0;
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			for(y = i*5; y < i*5+5; y++)
			{
				for(x = j*5; x < j*5+5; x++)
				{
					point->descriptor[index+0] += DX[y][x];
					point->descriptor[index+1] += DY[y][x];
					point->descriptor[index+2] += abs(DX[y][x]);
					point->descriptor[index+3] += abs(DY[y][x]);
				}
			}
			index+=4;
		}
	}

}

// modified FAST SURF descriptor
void wExtractFASTOpenSURF(IplImage* input, std::vector<windage::FeaturePoint>* keypoints)
{
//	IplImage *image = Integral(input);

	int N = keypoints->size();
	for(int k = 0; k < N; k++ )
    {
		double dir = (double)getOrientation(input, &((*keypoints)[k]));
		getDescriptor2(input, &((*keypoints)[k]));
	}

//	cvReleaseImage(&image);
}
