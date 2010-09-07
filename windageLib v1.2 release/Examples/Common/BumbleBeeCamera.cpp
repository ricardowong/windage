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

#define ALLOW_POINT_GREY
#ifdef ALLOW_POINT_GREY
#include "BumbleBeeCamera.h"
#include <stdlib.h>

BumbleBeeCamera::BumbleBeeCamera()
{
    m_digiclopsContext = NULL;
    m_leftImage = NULL;
    m_rightImage = NULL;
    m_leftColorImage = NULL;
    m_rightColorImage = NULL;
}
    
BumbleBeeCamera::~BumbleBeeCamera()
{
    clean();
}

void BumbleBeeCamera::init( int w, int h, unsigned char * left, unsigned char * right )
{
    DIGERROR( digiclopsCreateContext( &m_digiclopsContext ) );
    DIGERROR( digiclopsInitialize( m_digiclopsContext, 0 ) );
    DIGERROR( digiclopsGetTriclopsContextFromCamera( m_digiclopsContext, &m_triclopsContext ) );
    DIGERROR( digiclopsGetCameraInfoEx( m_digiclopsContext, &m_info ) );
    DIGERROR( digiclopsSetImageTypes( m_digiclopsContext, STEREO_IMAGE ) );
    DIGERROR( digiclopsSetImageResolution( m_digiclopsContext, DIGICLOPS_FULL ) );
    DIGERROR( digiclopsSetFrameRate( m_digiclopsContext, DIGICLOPS_FRAMERATE_100 ) );
    DIGERROR( digiclopsStart( m_digiclopsContext ) );

    m_hasColor = m_info.CameraType == DIGICLOPS_COLOR;

    if(m_hasColor) {
        m_leftColorImage = new unsigned char[ w*h*4 ];
        m_rightColorImage = new unsigned char[ w*h*4 ];
    }

    TRIERROR( triclopsSetResolutionAndPrepare(m_triclopsContext,h,w,
        imageHeight(),imageWidth()) );
    TRIERROR( triclopsSetImageBuffer(m_triclopsContext,m_leftImage,
        TriImg_RECTIFIED,TriCam_LEFT) );
    TRIERROR( triclopsSetImageBuffer(m_triclopsContext,m_rightImage,
        TriImg_RECTIFIED,TriCam_RIGHT) );

	TRIERROR( triclopsSetImageBuffer(m_triclopsContext,left,
        TriImg_RECTIFIED,TriCam_LEFT) );
    TRIERROR( triclopsSetImageBuffer(m_triclopsContext,right,
        TriImg_RECTIFIED,TriCam_RIGHT) );
   

   
}

void BumbleBeeCamera::SetColorBuffers(unsigned char * left, unsigned char * right)
{
	
     if(m_hasColor) 
	 {
        m_leftColorImage = left;
        m_rightColorImage = right;   
        TRIERROR( triclopsSetPackedColorImageBuffer(m_triclopsContext,
          TriCam_LEFT,(TriclopsPackedColorPixel*)m_leftColorImage) );
        TRIERROR( triclopsSetPackedColorImageBuffer(m_triclopsContext,
          TriCam_RIGHT,(TriclopsPackedColorPixel*)m_rightColorImage) );
     }

   
}

void BumbleBeeCamera::clean()
{
    if(m_digiclopsContext) {
        DIGERROR( digiclopsStop( m_digiclopsContext ) );
        DIGERROR( digiclopsDestroyContext( m_digiclopsContext ) );
        TRIERROR( triclopsDestroyContext( m_triclopsContext ) );
    }
 }

int BumbleBeeCamera::imageWidth() const
{
    switch(m_info.ImageSize) {
        case DIGICLOPS_320x240:
            return 320;
        case DIGICLOPS_640x480:
            return 640;
        case DIGICLOPS_1024x768:
            return 1024;
        default:
            return -1;
    }
}

int BumbleBeeCamera::imageHeight() const
{
    switch(m_info.ImageSize) {
        case DIGICLOPS_320x240:
            return 240;
        case DIGICLOPS_640x480:
            return 480;
        case DIGICLOPS_1024x768:
            return 768;
        default:
            return -1;
    }
}

int BumbleBeeCamera::imageBPP() const
{
    switch(m_info.CameraType) {
        case DIGICLOPS_BLACK_AND_WHITE:
            return 8;
        case DIGICLOPS_COLOR:
            return 24;
        default:
            return -1;
    }
}

void BumbleBeeCamera::capture()
{
    DIGERROR( digiclopsGrabImage( m_digiclopsContext ) );
}

void BumbleBeeCamera::download()
{
    TriclopsInput inputStereo,inputLeft,inputRight;
    TriclopsPackedColorImage outputLeft,outputRight;
    DIGERROR( digiclopsExtractTriclopsInput(m_digiclopsContext,STEREO_IMAGE,&inputStereo ) );
    if(m_hasColor) {
        DIGERROR( digiclopsExtractTriclopsInput(m_digiclopsContext,LEFT_IMAGE,&inputLeft ) );
        DIGERROR( digiclopsExtractTriclopsInput(m_digiclopsContext,RIGHT_IMAGE,&inputRight ) );
    }

    TRIERROR( triclopsRectify(m_triclopsContext,&inputStereo) );
    if(m_hasColor) {
        TRIERROR( triclopsRectifyPackedColorImage(m_triclopsContext,TriCam_LEFT,
            &inputLeft,&outputLeft) );
        TRIERROR( triclopsRectifyPackedColorImage(m_triclopsContext,TriCam_RIGHT,
            &inputRight,&outputRight) );
    }
 
}


#endif