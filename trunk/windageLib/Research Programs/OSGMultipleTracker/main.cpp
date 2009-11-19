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
#include <AugmentedReality/ARForOSG.h>

#include "NVideoLayer.h"
#include "OSGWrapper.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Logger* logging;
double fps = 0;
const int FPS_UPDATE_STEP = 30;
int fpsStep = 0;

// adaptive threshold
#define ADAPTIVE_THRESHOLD
int fastThreshold = 70;
const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 40;
const int ADAPTIVE_THRESHOLD_VALUE = 500;
const int THRESHOLD_STEP = 1;

const int OBJECT_COUNT = 2;
const int FIND_FEATURE_COUNT = 10;

CvCapture* capture;
windage::MultipleSURFTracker* tracker;
windage::ARForOSG* arTool;
IplImage* input;
IplImage* gray;

// for relation
std::vector<windage::Matrix3> rotationList;
std::vector<windage::Vector3> translationList;
std::vector<osg::MatrixTransform*> osgRelation;

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

using namespace windage;
osg::Matrixd ConvertMatrix(windage::Matrix4 matrix)
{
	osg::Matrixd osgMatrix;
	for(int y=0; y<4; y++)
	{
		for(int x=0; x<4; x++)
			osgMatrix(y, x) = matrix.m[y][x];
	}
	return osgMatrix;
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

	// draw tracking result
	int matchingCount = tracker->GetMatchedCount(0);
	bool updating = false;
	if(matchingCount > FIND_FEATURE_COUNT)
		updating = true;
	for(int i=0; i<tracker->GetTrackerCount(); i++)
	{
		int matchedCount = tracker->GetMatchedCount(i);
		if(matchedCount > FIND_FEATURE_COUNT)
		{
			tracker->DrawOutLine(input, i, true);
//			tracker->DrawInfomation(input, i, 100.0);

			if(updating)
			{
				rotationList[i] = GetRotation(tracker->GetCameraParameter(0), tracker->GetCameraParameter(i));
				translationList[i] = GetTranslation(tracker->GetCameraParameter(0), tracker->GetCameraParameter(i));

				windage::Matrix4 relationExtrinsic = CalculateRelationExtrinsicParameter(tracker->GetCameraParameter(0), rotationList[i], translationList[i]);
				osg::Matrixd tempMatrix = ConvertMatrix(windage::ARForOSG::CalculateModelViewMatrix(relationExtrinsic));
				osgRelation[i]->setMatrix(tempMatrix);
			}
		}
	}

	int featureCount = tracker->GetFeatureCount();
#ifdef ADAPTIVE_THRESHOLD
	if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
	else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

	// calculate fps
	fpsStep++;
	if(fpsStep > FPS_UPDATE_STEP)
	{
		fps = logging->calculateFPS()*(double)FPS_UPDATE_STEP;
		logging->updateTickCount();
		fpsStep = 0;
	}
	char message[100];
	sprintf(message, "FPS : %.2lf", fps);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 40), message);
	sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 60), message);
	sprintf(message, "Match count : %d", matchingCount);
	windage::Utils::DrawTextToImage(input, cvPoint(20, 80), message);

	arTool->SetModelViewMatrix();
	osg::Matrixd modelView = ConvertMatrix(arTool->GetModelViewMatrix());
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

	char filename[100];
	std::vector<IplImage*> trainingImage;
	for(int i=1; i<=OBJECT_COUNT; i++)
	{
		sprintf(filename, "reference%d_320.png", i);
		trainingImage.push_back(cvLoadImage(filename, 0));
	}

	tracker = new windage::MultipleSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, cvSize(8, 8), 3);
	tracker->SetDetectIntervalTime(1.0);
	tracker->SetPoseEstimationMethod(windage::LMEDS);
	tracker->SetOutlinerRemove(true);
	tracker->SetFeatureExtractThreshold(30);
	for(int i=0; i<trainingImage.size(); i++)
	{
		std::cout << "attatch reference image #" << i << std::endl;
		tracker->AttatchReferenceImage(trainingImage[i], 640, 480, 4.0, 8);
	}

	// for relation
	rotationList.resize(trainingImage.size());
	translationList.resize(trainingImage.size());

	// initialize ar tool
	arTool = new windage::ARForOSG();
	arTool->Initialize(WIDTH, HEIGHT);
	arTool->AttatchCameraParameter(tracker->GetCameraParameter(0)); // add base tracker

	// initialize OSG
	// create projection matrix
	arTool->SetProjectionMatrix();
	osg::Matrixd _proj = ConvertMatrix(arTool->GetProjectionMatrix());
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
	
	// attatch axis drawable
//*
	osg::ref_ptr<osg::Geode> geodeAxis = new osg::Geode();
	osg::Drawable* axis = CreateAxis(osg::Vec3(0, 0, 0), osg::Vec3(150, 0, 0), osg::Vec3(0, 150, 0), osg::Vec3(0, 0, 150));
	geodeAxis->addDrawable(axis);
//	localCoordinates->addChild(geodeAxis);
//*/

//*
	// attatch osg model
	osg::ref_ptr<osg::MatrixTransform> objectCoordinate = new osg::MatrixTransform();
	localCoordinates->addChild(objectCoordinate);

	osg::Matrixd scale;	scale.makeScale(50.0, 50.0, 50.0);
	osg::Matrixd translate;	translate.makeTranslate(0.0, 0.0, 3.0);
	objectCoordinate->postMult(translate);
	objectCoordinate->postMult(scale);
	objectCoordinate->addChild(LoadModel("model/cow.osg"));

	for(int i=0; i<OBJECT_COUNT; i++)
	{
		osgRelation.push_back(new osg::MatrixTransform());
		localCoordinates->addChild(osgRelation[i]);
	}
	for(int i=1; i<OBJECT_COUNT; i++)
	{
		osgRelation[i]->addChild(geodeAxis);
	}
//*/

	while (!viewer->done())
    {
		localCoordinates->setMatrix(GetTrackerCoordinate());
		viewer->frame();
    }

	cvReleaseCapture(&capture);
    return 0;
}
