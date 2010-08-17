/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include "Algorithms/OpenSURF/integral.h"

// Convert image to single channel 32F
IplImage *getGray(const IplImage *img)
{
  IplImage* gray8, * gray32;

  gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

  if( img->nChannels == 1 )
    gray8 = (IplImage *) cvClone( img );
  else {
    gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
    cvCvtColor( img, gray8, CV_BGR2GRAY );
  }

  cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );

  cvReleaseImage( &gray8 );
  return gray32;
}

//! Computes the integral image of image img.  Assumes source image to be a 
//! 32-bit floating point.  Returns IplImage of 32-bit float form.
IplImage *Integral(IplImage *source)
{
  // convert the image to single channel 32f
  IplImage *img = getGray(source);
  IplImage *int_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);

  // set up variables for data access
  int height = img->height;
  int width = img->width;
  int step = img->widthStep/sizeof(float);
  float *data   = (float *) img->imageData;  
  float *i_data = (float *) int_img->imageData;  

  // first row only
  float rs = 0.0f;
  for(int j=0; j<width; j++) 
  {
    rs += data[j]; 
    i_data[j] = rs;
  }

  // remaining cells are sum above and to the left
  for(int i=1; i<height; ++i) 
  {
    rs = 0.0f;
    for(int j=0; j<width; ++j) 
    {
      rs += data[i*step+j]; 
      i_data[i*step+j] = rs + i_data[(i-1)*step+j];
    }
  }

  // release the gray image
  cvReleaseImage(&img);

  // return the integral image
  return int_img;
}

