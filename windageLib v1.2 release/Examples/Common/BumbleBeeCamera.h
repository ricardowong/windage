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

#ifndef __POINTTGREYCAMERA_H__
#define __POINTGREYCAMERA_H__

// Point Grey Control Class
// Originally developed by David Gallup at UNC

#include <digiclops.h>
#include <triclops.h>
#include <windows.h>
#include <stdio.h>

class BumbleBeeCamera {
public:
    BumbleBeeCamera();
    ~BumbleBeeCamera();
    void init( int w, int h, unsigned char * left, unsigned char * right );
	void SetColorBuffers(unsigned char * left, unsigned char * right);
    void clean();

    int imageWidth() const;
    int imageHeight() const;
    int imageBPP() const;

    void capture();
    void download();
    unsigned char *leftImage() { return m_leftImage; }
    unsigned char *rightImage() { return m_rightImage; }
    bool hasColor() { return m_hasColor; }
    unsigned char *leftColorImage() { return m_leftColorImage; }
    unsigned char *rightColorImage() { return m_rightColorImage; }

private:
    DigiclopsContext m_digiclopsContext;
    TriclopsContext m_triclopsContext;
    DigiclopsInfoEx  m_info;
    unsigned char *m_leftImage;
    unsigned char *m_rightImage;
    bool m_hasColor;
    unsigned char *m_leftColorImage;
    unsigned char *m_rightColorImage;
};

#define DIGERROR(ee) \
{ \
    DigiclopsError e = ee; \
    if (e) { \
        printf("DigiclopsError: %s at %s %d\n", \
            digiclopsErrorToString(e),__FILE__,__LINE__); \
        exit(13); \
    } \
}

#define TRIERROR(ee) \
{ \
    TriclopsError e = ee; \
    if (e) { \
        printf("TriclopsError: %s at %s %d\n", \
            triclopsErrorToString(e),__FILE__,__LINE__); \
        exit(13); \
    } \
}


#endif
