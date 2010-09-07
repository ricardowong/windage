/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
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

#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/DisplaySettings>
#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osg/CoordinateSystemNode>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/CompositeViewer>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osgText/Text>

#include <osgManipulator/CommandManager>
#include "RecompositeDragger.h"

#include <highgui.h>

#include <windage.h>
#include <Coordinator/ARForOSG.h>
#include <ARforOSG/windageARforOSG.h>
#include <ARforOSG/NVideoLayer.h>

#include <iostream>

#include "../Common/OSG/OSGWrapper.h"

const char* FILE_NAME = "data/reconstruction-2010-03-29_18_28_38/reconstruction.txt";
const char* MODEL_FILE_NAME = "data/Model/Tank2/Leopard.osg";

#define AR_COORDINATION_MODE 1

const int WIDTH = 640;
const int HEIGHT = 480;
int selecteID = 0;
std::vector<windage::Calibration*> calibrationList;
std::vector<IplImage*> imageList;
IplImage* textureImage;
windage::Coordinator::ARForOSG* arTool = NULL;

osg::MatrixTransform* modelNode = NULL;

osg::Node* createHUD()
{
    osg::Geode* geode = new osg::Geode();
    
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osgText::Text* text = new  osgText::Text;
    geode->addDrawable( text );

    osg::Vec3 position(50.0f,50.0f,0.0f);
    text->setPosition(position);
    text->setText("Use the Tab key to switch between the trackball and pick modes.");

    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));

    // set the view matrix    
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    camera->addChild(geode);
    
    return camera;
}

osg::Node* addDraggerToScene(osg::Node* scene, osgManipulator::CommandManager* cmdMgr)
{
    scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osgManipulator::Selection* selection = new osgManipulator::Selection;
    selection->addChild(scene);

	osgManipulator::RecompositeDragger* dragger = new osgManipulator::RecompositeDragger();
	dragger->setupDefaultGeometry();

    osg::Group* root = new osg::Group;
    root->addChild(dragger);
    root->addChild(selection);
    root->addChild(createHUD());

    float scale = scene->getBound().radius() * 1.0;
    dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                       osg::Matrix::translate(scene->getBound().center()));

    cmdMgr->connect(*dragger, *selection);

    return root;
}

osg::Node* createModelScene(osgManipulator::CommandManager* cmdMgr)
{ 
	// for model
	modelNode = new osg::MatrixTransform;
	modelNode->addChild(addDraggerToScene(LoadModel(MODEL_FILE_NAME), cmdMgr));
    return modelNode;
}

osg::Node* createReconstructionScene(std::vector<windage::FeaturePoint>* referenceRepository)
{ 
	// for reconstruction points
	int count = referenceRepository->size();

	osg::Geode* reconstructionGeode = new osg::Geode;
	osg::Geometry* geometry = new osg::Geometry();
	osg::Vec3dArray* vertices = new osg::Vec3dArray(count);
	osg::Vec4Array* colors = new osg::Vec4Array(count);
	for(int i=0; i<count; i++)
	{
		windage::Vector3 point = (*referenceRepository)[i].GetPoint();
		(*vertices)[i] = osg::Vec3d(point.x, point.y, point.z);
		
		CvScalar color = (*referenceRepository)[i].GetColor();
		(*colors)[i] = osg::Vec4d(color.val[2]/255.0, color.val[1]/255.0, color.val[0]/255.0, 1.0);
	}
	geometry->setVertexArray(vertices);
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, count));
	geometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);

	reconstructionGeode->addDrawable(geometry);
	
    return reconstructionGeode;
}

class PickModeHandler : public osgGA::GUIEventHandler
{
    public:
        enum Modes
        {
            VIEW = 0,
            PICK
        };

        PickModeHandler(): _mode(VIEW), _activeDragger(0)
        {
        }        
        
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (!view) return false;

			// keyboard event
			switch(ea.getKey())
			{
			case osgGA::GUIEventAdapter::KEY_Tab:
				if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && _activeDragger == 0)
				{
					_mode = ! _mode;
				}
				break;
			case osgGA::GUIEventAdapter::KEY_Up:
			case osgGA::GUIEventAdapter::KEY_Right:
				selecteID++;
				if(selecteID >= imageList.size()) selecteID = 0;
				cvCopyImage(imageList[selecteID], textureImage);
				break;
			case osgGA::GUIEventAdapter::KEY_Down:
			case osgGA::GUIEventAdapter::KEY_Left:
				selecteID--;
				if(selecteID < 0) selecteID = imageList.size() - 1;
				cvCopyImage(imageList[selecteID], textureImage);
				break;
			case 'c':
			case 'C':
//*
				osg::Vec3d trans = ((osg::MatrixTransform*)modelNode->getChild(0)->asGroup()->getChild(0))->getMatrix().getTrans();
				osg::Vec3d rotat = ((osg::MatrixTransform*)modelNode->getChild(0)->asGroup()->getChild(0))->getMatrix().getRotate().asVec3();
				osg::Vec3d scale = ((osg::MatrixTransform*)modelNode->getChild(0)->asGroup()->getChild(0))->getMatrix().getScale();

				std::cout << "Translation : " << trans[0] << ", " << trans[1] << ", " << trans[2] << std::endl;
				std::cout << "Rotation : " << rotat[0] << ", " << rotat[1] << ", " << rotat[2] << std::endl;
				std::cout << "Scale : " << scale[0] << ", " << scale[1] << ", " << scale[2] << std::endl;
//*/
				break;
			}
            
            if (VIEW == _mode) return false;

			// mouse event
            switch (ea.getEventType())
            {
                case osgGA::GUIEventAdapter::PUSH:
                {
                    osgUtil::LineSegmentIntersector::Intersections intersections;
                    _pointer.reset();

                    if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
                    {
                        _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());

                        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                            hitr != intersections.end();
                            ++hitr)
                        {
                            _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
                        }
                        for (osg::NodePath::iterator itr = _pointer._hitList.front().first.begin();
                             itr != _pointer._hitList.front().first.end();
                             ++itr)
                        {
                            osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(*itr);
                            if (dragger)
                            {

                                dragger->handle(_pointer, ea, aa);
                                _activeDragger = dragger;
                                break;
                            }                   
                        }
                    }
                }
                case osgGA::GUIEventAdapter::DRAG:
                case osgGA::GUIEventAdapter::RELEASE:
                {
                    if (_activeDragger)
                    {
                        _pointer._hitIter = _pointer._hitList.begin();
                        _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());

                        _activeDragger->handle(_pointer, ea, aa);
                    }
                    break;
                }
        default:
            break;
            }

            if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
            {
                _activeDragger = 0;
                _pointer.reset();
            }

            return true;
        }
        
    private:
        unsigned int _mode;
        osgManipulator::Dragger* _activeDragger;
        osgManipulator::PointerInfo _pointer;
};

osg::Matrixd GetTrackerCoordinate()
{
	// camera frame grabbing
	IplImage* backgroundImage;

	arTool->AttatchCameraParameter(calibrationList[selecteID]);
	arTool->SetModelViewMatrix();
	osg::Matrixd modelView;
	modelView.identity();
	modelView = windageARTool::ForOSG::ConvertOSGMatrix(arTool->GetModelViewMatrix());

	return modelView;
}

int main( int argc, char **argv )
{
	osg::Group* root = new osg::Group;
	osg::ref_ptr<osg::Projection>		projectionMatrix;
	osg::ref_ptr<osg::MatrixTransform>	modelViewMatrix;

	osg::ref_ptr<osg::Group>	sceneGroup = new osg::Group();
	osg::ref_ptr<osg::Group>	foregroundGroup = new osg::Group();
	osg::ref_ptr<CNVideoLayer>	videoBackground;
	osg::ref_ptr<osg::Image>	cameraImage = new osg::Image();

	// for reconstruction data
	std::vector<windage::ReconstructionPoint> reconstructionPoints;
	std::vector<std::string> filenameList;

	// load data
	std::cout << "load reconstruction datas" << std::endl;
	std::cout << std::endl;

	windage::Reconstruction::Loader* loader = new windage::Reconstruction::Loader();
	loader->AttatchCalibration(&calibrationList);
	loader->AttatchFilename(&filenameList);
	loader->AttatchReconstructionPoints(&reconstructionPoints);
	loader->DoLoad(FILE_NAME);

	for(unsigned int i=0; i<filenameList.size(); i++)
	{
		imageList.push_back(cvLoadImage(filenameList[i].c_str()));
	}

	std::cout << std::endl;
	std::cout << "complete load reconstruction datas" << std::endl;
	std::cout << "reconstruction data count : " << reconstructionPoints.size() << std::endl;
	std::cout << std::endl;

	// for tracking
	std::vector<windage::FeaturePoint> referenceRepository;
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		windage::FeaturePoint feature = reconstructionPoints[i].GetFeature(0);
		windage::Vector4 point = reconstructionPoints[i].GetPoint();
		feature.SetPoint(windage::Vector3(point.x, point.y, point.z));
		feature.SetObjectID(0);
		feature.SetRepositoryID(i);

		referenceRepository.push_back(feature);
	}

	// for rendering
	arTool = new windage::Coordinator::ARForOSG();
	arTool->Initialize(WIDTH, HEIGHT);
	arTool->AttatchCameraParameter(calibrationList[0]);
	textureImage = cvCloneImage(imageList[0]);

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
	cameraImage->setImage(WIDTH, HEIGHT, 1, GL_BGRA, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char*)textureImage->imageData, osg::Image::NO_DELETE);	
	videoBackground = new CNVideoLayer(cameraImage.get(), 1);
	videoBackground->init();
#if AR_COORDINATION_MODE
	sceneGroup->addChild(videoBackground.get());
#endif

	// create foregruond scene
	foregroundGroup->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");
	foregroundGroup->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	sceneGroup->addChild(foregroundGroup.get());

	// tracker transform
	osg::ref_ptr<osg::MatrixTransform> localCoordinates;
	localCoordinates = new osg::MatrixTransform();
	foregroundGroup->addChild(localCoordinates.get());

    // construct the viewer.
    osgViewer::Viewer* viewer;
	viewer = new osgViewer::Viewer();
	viewer->setUpViewInWindow(10, 40, 640, 480 );
	
	// create a command manager
    osg::ref_ptr<osgManipulator::CommandManager> cmdMgr = new osgManipulator::CommandManager;

    // if no model has been successfully loaded report failure.
	localCoordinates->addChild(createModelScene(cmdMgr.get()));
	localCoordinates->addChild(createReconstructionScene(&referenceRepository));

    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(root);
    
    // pass the loaded scene graph to the viewer
	viewer->setSceneData(root);
    viewer->addEventHandler(new PickModeHandler());
	viewer->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	viewer->realize();

#if AR_COORDINATION_MODE
	while(!viewer->done())
	{
		localCoordinates->setMatrix(GetTrackerCoordinate());
		viewer->frame();
	}
#else
	viewer->run();
#endif

    return 0;
}

