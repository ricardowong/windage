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

#ifndef OSGMANIPULATOR_SCALE_UNIFORM_DRAGGER
#define OSGMANIPULATOR_SCALE_UNIFORM_DRAGGER 1

#include <osgManipulator/Dragger>
#include <osgManipulator/Projector>
#include <osgManipulator/CommandManager>

namespace osgManipulator {

/**
 * Dragger for performing 1D scaling.
 */
class ScaleUniformDragger : public Dragger
{
    public:

        enum ScaleMode
        {
            SCALE_WITH_ORIGIN_AS_PIVOT = 0,
            SCALE_WITH_OPPOSITE_HANDLE_AS_PIVOT
        };

        ScaleUniformDragger(ScaleMode scaleMode=SCALE_WITH_ORIGIN_AS_PIVOT);

        /**
         * Handle pick events on dragger and generate TranslateInLine commands.
         */
        virtual bool handle(const PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

        /** Setup default geometry for dragger. */
        void setupDefaultGeometry();

        /** Set/Get min scale for dragger. */
        inline void  setMinScale(double min) { _minScale = min; }
        inline double getMinScale() const    { return _minScale; }

        /** Set/Get color for dragger. */
        inline void setColor(const osg::Vec4& color) { _color = color; setMaterialColor(_color,*this); }
        inline const osg::Vec4 getColor() const { return _color; }

        /**
         * Set/Get pick color for dragger. Pick color is color of the dragger
         * when picked. It gives a visual feedback to show that the dragger has
         * been picked.
         */
        inline void setPickColor(const osg::Vec4& color) { _pickColor = color; }
        inline const osg::Vec4 getPickColor() const { return _pickColor; }

        /** Set/Get left and right handle nodes for dragger. */
        inline void setLeftHandleNode (osg::Node& node) { _leftHandleNode = &node; }
        inline void setRightHandleNode(osg::Node& node) { _rightHandleNode = &node; }
        inline osg::Node* getLeftHandleNode()  { return _leftHandleNode.get(); }
        inline osg::Node* getRightHandleNode() { return _rightHandleNode.get(); }

        /** Set left/right handle position. */
        inline void  setLeftHandlePosition(double pos)  { _projector->getLineStart() = osg::Vec3d(pos,0.0,0.0); }
        inline double getLeftHandlePosition() const     { return _projector->getLineStart()[0]; }
        inline void  setRightHandlePosition(double pos) { _projector->getLineEnd() = osg::Vec3d(pos,0.0,0.0); }
        inline double getRightHandlePosition()          { return _projector->getLineEnd()[0]; }

    protected:

        virtual ~ScaleUniformDragger();

        osg::ref_ptr< LineProjector >   _projector;
        osg::Vec3d                      _startProjectedPoint;
        osg::Vec3d                      _scaleCenter;
        double                          _minScale;

        osg::ref_ptr< osg::Node >       _leftHandleNode;
        osg::ref_ptr< osg::Node >       _rightHandleNode;

        osg::Vec4                       _color;
        osg::Vec4                       _pickColor;

        ScaleMode                       _scaleMode;
};


}

#endif
