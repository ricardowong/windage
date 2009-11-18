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

#include <windows.h>

#include <osg/ShapeDrawable>

#include <osg/Geometry>
#include <osg/DisplaySettings>
#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osgDB/ReadFile>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>

#include "NVideoLayer.h"

#include <iostream>
#include <highgui.h>
#include <windage.h>


const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

// adaptive threshold
#define ADAPTIVE_THRESHOLD
int fastThreshold = 70;
const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 40;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

CvCapture* capture;
windage::ModifiedSURFTracker* tracker;
IplImage* input;
IplImage* gray;

// for OSG
osgViewer::Viewer* viewer;
osg::ref_ptr<osg::MatrixTransform>	localCoordinates;

using namespace windage;
windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, 640, 480, 4.0, 8);
	tracker->SetPoseEstimationMethod(windage::RANSAC);
	tracker->SetOutlinerRemove(true);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 5, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

// class to handle events with a pick
class PickHandler : public osgGA::GUIEventHandler
{
public: 
	PickHandler() {}
	~PickHandler() {}
    
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
	{
		switch(ea.getEventType())
		{
			case(osgGA::GUIEventAdapter::KEYDOWN):
			{
				switch(ea.getKey())
				{
				case 'q':
				case 'Q':
					cvReleaseCapture(&capture);
					exit(0);
					break;
				}
			}
			break;
			default:
				return false;
				break;
		}	
	}
};

osg::Matrixd CreateProjectionMatrix(double fx, double fy, double cx, double cy, double width, double height)
{
	osg::Matrixd _proj;
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++) _proj(i, j) = 0.0;
	}
	_proj(0,0)  =  2.0 * fx / width;
	_proj(1,1)  =  2.0 * fy / height;
	_proj(2,0)  = -2.0 * cx / width + 1.0;
	_proj(2,1)  =  2.0 * cy / height - 1.0;
	_proj(2,3)  = -1.0;

	float z_far  = 10000;
	float z_near = 0.01;
	_proj(2,2) = (z_far+z_near)/(z_near-z_far);
	_proj(3,2) = -2.0 * z_far * z_near / (z_far-z_near);

	return _proj;
}

osg::Matrixd ChangeCalibrationToOSGMatrix(CvMat *_RT)
{
	osg::Matrixd RT;
	double crt[16];

	crt[0] = cvmGet(_RT, 0, 0);
	crt[4] = cvmGet(_RT, 0, 1);
	crt[8] = cvmGet(_RT, 0, 2);
	crt[12]= cvmGet(_RT, 0, 3);

	crt[1] = -cvmGet(_RT, 1, 0);
	crt[5] = -cvmGet(_RT, 1, 1);
	crt[9] = -cvmGet(_RT, 1, 2);
	crt[13]= -cvmGet(_RT, 1, 3);

	crt[2] = -cvmGet(_RT, 2, 0);
	crt[6] = -cvmGet(_RT, 2, 1);
	crt[10]= -cvmGet(_RT, 2, 2);
	crt[14]= -cvmGet(_RT, 2, 3);

	crt[3] = 0;
	crt[7] = 0;
	crt[11]= 0;
	crt[15]= 1;

	RT.set(crt);
	return RT;
}
osg::Matrixd GetTrackerCoordinate()
{
	// camera frame grabbing
	IplImage* grabFrame = cvQueryFrame(capture);
	tracker->GetCameraParameter()->Undistortion(grabFrame, input);
	cvFlip(input, input);
	cvCvtColor(input, gray, CV_BGRA2GRAY);

	// call tracking algorithm
	tracker->SetFeatureExtractThreshold(fastThreshold);
	tracker->UpdateCameraPose(gray);
	tracker->DrawOutLine(input, true);
//	tracker->DrawDebugInfo(input);

#ifdef ADAPTIVE_THRESHOLD
	int featureCount = tracker->GetFeatureCount();
	if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
	else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

	CvMat* extrinsic = tracker->GetCameraParameter()->GetExtrinsicMatrix();
	return ChangeCalibrationToOSGMatrix(extrinsic);
}

osg::Drawable* CreateAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir)
{
	// set up the Geometry.
	osg::Geometry* geom = new osg::Geometry;

	osg::Vec3Array* coords = new osg::Vec3Array(6);
	(*coords)[0] = corner;
	(*coords)[1] = corner+xdir;
	(*coords)[2] = corner;
	(*coords)[3] = corner+ydir;
	(*coords)[4] = corner;
	(*coords)[5] = corner+zdir;

	geom->setVertexArray(coords);

	osg::Vec4 x_color(1.0f,0.0f,0.0f,1.0f);
	osg::Vec4 y_color(0.0f,1.0f,0.0f,1.0f);
	osg::Vec4 z_color(0.0f,0.0f,1.0f,1.0f);

	osg::Vec4Array* color = new osg::Vec4Array(6);
	(*color)[0] = x_color;
	(*color)[1] = x_color;
	(*color)[2] = y_color;
	(*color)[3] = y_color;
	(*color)[4] = z_color;
	(*color)[5] = z_color;

	geom->setColorArray(color);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

	osg::StateSet* stateset = new osg::StateSet;
	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(10.0f);
	stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	geom->setStateSet(stateset);

	return geom;
}

DWORD WINAPI WorkerThread(LPVOID)
{
	while (!viewer->done())
    {
		localCoordinates->setMatrix(GetTrackerCoordinate());
		viewer->frame();
		cvWaitKey(10);
    }
	return 0;
}

int main(int argc, char ** argv )
{
    osg::ArgumentParser arguments(&argc, argv);
    osg::ref_ptr<osg::Group> root = new osg::Group();
	osg::ref_ptr<osg::Projection>		projectionMatrix;
	osg::ref_ptr<osg::MatrixTransform>	modelViewMatrix;

	osg::ref_ptr<osg::Group>	sceneGroup = new osg::Group();
	osg::ref_ptr<osg::Group>	foregroundGroup = new osg::Group();
	osg::ref_ptr<CNVideoLayer>	videoBackground;
	osg::ref_ptr<osg::Image>	cameraImage = new osg::Image();
	
	// initialize tracker
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	gray = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	IplImage* referenceImage = cvLoadImage("reference1_320.png", 0);
	tracker = CreateTracker(referenceImage, 0);
	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);

	// initialize OSG
	// create projection matrix
	osg::Matrixd _proj = CreateProjectionMatrix(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], (double)WIDTH, (double)HEIGHT);
	projectionMatrix = new osg::Projection(_proj);
	modelViewMatrix  = new osg::MatrixTransform();

	// setup scene graph
	root->addChild(projectionMatrix.get());
	projectionMatrix->addChild(modelViewMatrix.get());
	modelViewMatrix->addChild(sceneGroup.get());

	// create background scene
	cameraImage->setImage(WIDTH, HEIGHT, 1, GL_BGRA, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char*)input->imageData, osg::Image::NO_DELETE);	
	videoBackground = new CNVideoLayer(cameraImage.get(), 1);
	videoBackground->init();
	sceneGroup->addChild(videoBackground.get());
//	sceneGroup->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");

	// create foregruond scene
	foregroundGroup->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");
	foregroundGroup->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	sceneGroup->addChild(foregroundGroup.get());

	// tracker transform
	localCoordinates = new osg::MatrixTransform();
	foregroundGroup->addChild(localCoordinates.get());
	
	// create viewer
	viewer = new osgViewer::Viewer();
	viewer->addEventHandler(new osgViewer::StatsHandler);

	viewer->setUpViewInWindow(10, 40, 640, 480 );
	viewer->addEventHandler(new PickHandler());
	viewer->setSceneData(root.get());
	viewer->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	viewer->realize();

	osg::ref_ptr<osg::MatrixTransform> objectCoordinate = new osg::MatrixTransform();
	localCoordinates->addChild(objectCoordinate);

	// attatch axis drawable
	osg::ref_ptr<osg::Geode> geodeAxis = new osg::Geode();
	osg::Drawable* axis = CreateAxis(osg::Vec3(0, 0, 0), osg::Vec3(300, 0, 0), osg::Vec3(0, 300, 0), osg::Vec3(0, 0, 300));
	geodeAxis->addDrawable(axis);
	objectCoordinate->addChild(geodeAxis);

	while (!viewer->done())
    {
		localCoordinates->setMatrix(GetTrackerCoordinate());
		viewer->frame();
    }

	cvReleaseCapture(&capture);
    return 0;
}
