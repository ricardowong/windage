#pragma once
/**
 * Wrapper class for controlling PGR cameras such as flea, dragonfly..
 *
*/

// opencv
#include "cxcore.h"
#include "cv.h"
#include "highgui.h"

// pgr
#include "PGRFlyCapture.h"
#include "PGRFlyCapturegui.h"

// etc
#include <string>

class CPGRCamera
{
public:
	CPGRCamera(void);
	~CPGRCamera(void);

	void open();
	void close();
	void start();
	void stop();
	void update();

	void releaseImage() {};

	int xsize, ysize, pixelsize, framerate;
	IplImage *m_Image;

	IplImage *GetIPLImage()
	{
		return m_Image;
	}

private:
	void handleError(std::string function);
	void handleGuiError(std::string function);
	void reportCameraInfo();

	FlyCaptureContext _context;
	FlyCaptureError _error;

	CameraGUIContext _gui_context;
	CameraGUIError _gui_error;

	unsigned char* _buffer;
};
