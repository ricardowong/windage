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

#ifndef _FAST_H_
#define _FAST_H_

/** @cond */

typedef struct { int x, y; } xy;
typedef unsigned char byte;

int corner_score(const byte*  imp, const int *pointer_dir, int barrier);

/*void fast_nonmax(const BasicImage<byte>& im, const vector<ImageRef>& corners, int barrier, vector<ReturnType>& nonmax_corners)*/
xy*  xfast_nonmax(const byte* im, int xsize, int ysize, xy* corners, int numcorners, int barrier, int* numnx);

/*void fast_nonmax(const BasicImage<byte>& im, const vector<ImageRef>& corners, int barrier, vector<ReturnType>& nonmax_corners)*/
xy*  fast_nonmax(const byte* im, int xsize, int ysize, xy* corners, int numcorners, int barrier, int* numnx);

xy* fast_corner_detect_9(const byte* im, int xsize, int ysize, int barrier, int* num);
xy* fast_corner_detect_10(const byte* im, int xsize, int ysize, int barrier, int* num);
xy* fast_corner_detect_11(const byte* im, int xsize, int ysize, int barrier, int* num);
xy* fast_corner_detect_12(const byte* im, int xsize, int ysize, int barrier, int* num);

#endif
