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

#include <iostream>
#include <highgui.h>

#include <windage.h>
#include <Coordinator/ARForOSG.h>
#include <ARforOSG/windageARforOSG.h>
#include <ARforOSG/NVideoLayer.h>
#include "OSGWrapper.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

windage::Logger* logging;
double fps = 0;
const int FPS_UPDATE_STEP = 30;
int fpsStep = 0;

// adaptive threshold
#define ADAPTIVE_THRESHOLD 1
int beforeCount = -1;
int fastThreshold = 30;
const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 10;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

bool flip = true;
CvCapture* capture;
windage::Frameworks::PlanarObjectTracking* tracker = NULL;
windage::Coordinator::ARForOSG* arTool = NULL;
IplImage* input;
IplImage* gray;

CvVideoWriter* writer;
#define SAVE_RENDERING_IMAGE

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
				case 'f':
				case 'F':
					flip = !flip;
					break;
				case 'q':
				case 'Q':
					if(writer) cvReleaseVideoWriter(&writer);
					if(capture) cvReleaseCapture(&capture);
					exit(0);
					break;
				case ' ':
					tracker->GetDetector()->SetThreshold(15.0);
					tracker->AttatchReferenceImage(gray);
					tracker->TrainingReference(4.0, 8);
					tracker->GetDetector()->SetThreshold(fastThreshold);
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

windage::Frameworks::PlanarObjectTracking* CreateTracker()
{
	windage::Frameworks::PlanarObjectTracking* tracker = new windage::Frameworks::PlanarObjectTracking();

	windage::Calibration* calibration = new windage::Calibration();
	windage::Algorithms::FeatureDetector* detector = new windage::Algorithms::WSURFdetector();
	windage::Algorithms::SearchTree* searchtree = new windage::Algorithms::FLANNtree();
	windage::Algorithms::OpticalFlow* opticalflow = new windage::Algorithms::OpticalFlow();
	windage::Algorithms::HomographyEstimator* estimator = new windage::Algorithms::ProSACestimator();
	windage::Algorithms::OutlierChecker* checker = new windage::Algorithms::OutlierChecker();
	windage::Algorithms::HomographyRefiner* refiner = new windage::Algorithms::LMmethod();

	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3],
							intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	detector->SetThreshold(30.0);
	searchtree->SetRatio(0.7);
	opticalflow->Initialize(WIDTH, HEIGHT, cvSize(8, 8), 3);
	estimator->SetReprojectionError(5.0);
	checker->SetReprojectionError(5.0);
	refiner->SetMaxIteration(10);

	tracker->AttatchCalibration(calibration);
	tracker->AttatchDetetor(detector);
	tracker->AttatchMatcher(searchtree);
	tracker->AttatchTracker(opticalflow);
	tracker->AttatchEstimator(estimator);
	tracker->AttatchChecker(checker);
	tracker->AttatchRefiner(refiner);

	tracker->SetDitectionRatio(15);
	tracker->Initialize(WIDTH, HEIGHT, (double)WIDTH, (double)HEIGHT);

	return tracker;
}

osg::Matrixd GetTrackerCoordinate()
{
	// camera frame grabbing
	IplImage* grabFrame = cvQueryFrame(capture);
	if(flip)
		cvFlip(grabFrame, grabFrame);
	tracker->GetCameraParameter()->Undistortion(grabFrame, input);
	cvCvtColor(input, gray, CV_BGRA2GRAY);

	// call tracking algorithm
	tracker->GetDetector()->SetThreshold(fastThreshold);
	tracker->UpdateCamerapose(gray);
	tracker->DrawDebugInfo(input);

	int featureCount = tracker->GetDetector()->GetKeypointsCount();
	int matchingCount = tracker->GetMatchingCount();
#ifdef ADAPTIVE_THRESHOLD
	if(beforeCount != featureCount)
	{
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
		tracker->GetDetector()->SetThreshold(fastThreshold);
	}
	beforeCount = featureCount;
#endif

	// calculate fps
	fpsStep++;
	if(fpsStep > FPS_UPDATE_STEP)
	{
		fps = logging->calculateFPS()*(double)FPS_UPDATE_STEP;
		logging->updateTickCount();
		fpsStep = 0;
	}
	
	arTool->SetModelViewMatrix();

	osg::Matrixd modelView;
	modelView.identity();
	if(matchingCount > 10)
	{
		tracker->DrawOutLine(input, true);
		modelView = windageARTool::ForOSG::ConvertOSGMatrix(arTool->GetModelViewMatrix());
	}

	char message[100];
	sprintf_s(message, "FPS : %.2lf", fps);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 40), 0.7, message);
	sprintf_s(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 60), 0.7, message);
	sprintf_s(message, "Match count : %d", matchingCount);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 80), 0.7, message);
	sprintf_s(message, "Press 'F' to flip image");
	windage::Utils::DrawTextToImage(input, cvPoint(WIDTH-170, HEIGHT-25), 0.5, message);

	return modelView;
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

	// for checking FPS
	logging = new windage::Logger(&std::cout);
	logging->updateTickCount();

	// initialize tracker
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	input = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	gray = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

//	IplImage* referenceImage = cvLoadImage("reference1_320.png", 0);
	tracker = CreateTracker();

	// initialize ar tool
	arTool = new windage::Coordinator::ARForOSG();
	arTool->Initialize(WIDTH, HEIGHT);
	arTool->AttatchCameraParameter(tracker->GetCameraParameter());

	// initialize OSG
	// create projection matrix
	arTool->SetProjectionMatrix();
	osg::Matrixd _proj = windageARTool::ForOSG::ConvertOSGMatrix(arTool->GetProjectionMatrix());
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

	// create foregruond scene
	foregroundGroup->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");
	foregroundGroup->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	sceneGroup->addChild(foregroundGroup.get());

	// tracker transform
	osg::ref_ptr<osg::MatrixTransform> localCoordinates;
	localCoordinates = new osg::MatrixTransform();
	foregroundGroup->addChild(localCoordinates.get());
	
	// create viewer
	// for OSG
	osgViewer::Viewer* viewer;
	viewer = new osgViewer::Viewer();
	viewer->addEventHandler(new osgViewer::StatsHandler);

	viewer->setUpViewInWindow(10, 40, 640, 480 );
	viewer->addEventHandler(new PickHandler());
	viewer->setSceneData(root.get());
	viewer->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	viewer->realize();
	
	// attatch osg model
	osg::ref_ptr<osg::MatrixTransform> objectCoordinate = new osg::MatrixTransform();
	localCoordinates->addChild(objectCoordinate);

	double scaleFactor = 50;
	double x = 0.0;
	double y = 0.0;
	double z = 3.0;
	osg::Matrixd scale;	scale.makeScale(scaleFactor, scaleFactor, scaleFactor);
	osg::Matrixd translate;	translate.makeTranslate(x, y, z);
	objectCoordinate->postMult(translate);
	objectCoordinate->postMult(scale);
	objectCoordinate->addChild(LoadModel("model/cow.osg"));

#ifdef SAVE_RENDERING_IMAGE
	    IplImage* saveImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
        IplImage* saveTempImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 4);
        bool saving = false;
        writer = cvCreateVideoWriter("rendering.avi", CV_FOURCC_DEFAULT, 30, cvSize(saveImage->width, saveImage->height), 1);

        osg::Image* osgImage = new osg::Image();
        osgImage->allocateImage(saveImage->width, saveImage->height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        viewer->getCamera()->attach(osg::Camera::COLOR_BUFFER, osgImage);
#endif

	while (!viewer->done())
    {
		localCoordinates->setMatrix(GetTrackerCoordinate());
		viewer->frame();

#ifdef SAVE_RENDERING_IMAGE
                std::cout << "read buffer" << std::endl;
                osgImage->readPixels(0, 0, saveImage->width, saveImage->height, GL_RGBA, GL_UNSIGNED_BYTE);
                memcpy(saveTempImage->imageData, osgImage->data(), sizeof(char)*saveImage->width*saveImage->height*4);
                cvCvtColor(saveTempImage, saveImage, CV_RGBA2BGR);
                cvFlip(saveImage, saveImage);
                
                if(writer) cvWriteFrame(writer, saveImage);
#endif
    }

	cvReleaseCapture(&capture);
    return 0;
}
