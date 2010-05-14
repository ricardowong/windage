/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/
//osgManipulator - Copyright (C) 2007 Fugro-Jason B.V.

#include "RecompositeDragger.h"

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Quat>

using namespace osgManipulator;

RecompositeDragger::RecompositeDragger()
{
    tDragger = new osgManipulator::TranslateAxisDragger();
    addChild(tDragger.get());
    addDragger(tDragger.get());

	rDragger = new osgManipulator::TrackballDragger();
    addChild(rDragger.get());
    addDragger(rDragger.get());

	sDragger = new osgManipulator::ScaleUniformDragger();
    addChild(sDragger.get());
    addDragger(sDragger.get());

    setParentDragger(getParentDragger());
}
       
RecompositeDragger::~RecompositeDragger()
{
}

void RecompositeDragger::setupDefaultGeometry()
{
	tDragger->setupDefaultGeometry();
	rDragger->setupDefaultGeometry();
	double scale = 0.5;
	rDragger->setMatrix(osg::Matrix::scale(scale, scale, scale));
	sDragger->setupDefaultGeometry();
}
