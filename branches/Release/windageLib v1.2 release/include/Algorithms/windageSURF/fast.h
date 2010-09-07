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
 * @file	fast.h
 * @author	Woonhyuk Baek
 * @version 2.0
 * @date	2010.02.04
 * @brief	wrapping and modifed FAST cornder detector
 */

#ifndef _FAST_H_
#define _FAST_H_

#define STEP_SIZE_X 1
#define STEP_SIZE_Y 2

typedef struct { int x, y; } xy; 
typedef unsigned char byte;

int fast9_corner_score(const byte* p, const int pixel[], int bstart);
int fast10_corner_score(const byte* p, const int pixel[], int bstart);
int fast11_corner_score(const byte* p, const int pixel[], int bstart);
int fast12_corner_score(const byte* p, const int pixel[], int bstart);

xy* fast9_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int stepx=STEP_SIZE_X, int stepy=STEP_SIZE_Y);
xy* fast10_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int stepx=STEP_SIZE_X, int stepy=STEP_SIZE_Y);
xy* fast11_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int stepx=STEP_SIZE_X, int stepy=STEP_SIZE_Y);
xy* fast12_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int stepx=STEP_SIZE_X, int stepy=STEP_SIZE_Y);

int* fast9_score(const byte* i, int stride, xy* corners, int num_corners, int b);
int* fast10_score(const byte* i, int stride, xy* corners, int num_corners, int b);
int* fast11_score(const byte* i, int stride, xy* corners, int num_corners, int b);
int* fast12_score(const byte* i, int stride, xy* corners, int num_corners, int b);

xy* fast9_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
xy* fast10_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
xy* fast11_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
xy* fast12_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

xy* nonmax_suppression(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax);

#endif // _FAST_H_