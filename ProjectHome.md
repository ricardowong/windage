# windage's personal project #

## [windage library](http://code.google.com/p/windage/source/browse/#svn/trunk/windageLib) ##
> windage library is C++ library for augmented reality.
    * Source code is available under the GNU General Public License. In short, if you distribute a software that uses windageLib-demo, you have to distribute it under GPL with the source code. The code that can be dowloaded from this site should **only be used for research and evaluation purposes**. Another option is to contact us to purchase a commercial license.

### Functions ###
  * Demo : http://www.youtube.com/watch?v=orf5M4K01RY

  * Tracking Functions
    1. Real-time Natural Feature Tracking
> > > ![http://windage.googlecode.com/svn/trunk/Document/images/Demo_ModifiedSURF.png](http://windage.googlecode.com/svn/trunk/Document/images/Demo_ModifiedSURF.png)
    1. Real-time Multiple Object Tracking
> > > ![http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultipleObjectTracking.png](http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultipleObjectTracking.png)
    1. Multi-method Tracking
> > > ![http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultimethodTracking.png](http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultimethodTracking.png)

  * Augmented Reality Functions
    1. Augmented Reality
> > > ![http://windage.googlecode.com/svn/trunk/Document/images/Demo_AugmentedReality.png](http://windage.googlecode.com/svn/trunk/Document/images/Demo_AugmentedReality.png)
    1. Multiple Object Augmented Reality
> > > ![http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultipleObjectAR.png](http://windage.googlecode.com/svn/trunk/Document/images/Demo_MultipleObjectAR.png)

### Sample Applications ###

> [Sample Application Links](SampleApplications.md)

### Release Note ###
  * Release Version 1.0
    1. OpenCV 1.2 to OpenCV 2.0
    1. Refactoring
      * Modifiy intreface
      * Remove legacy codes
    1. Add test program
      * check algorithms
      * check memory release (not accurately)
    1. Multiple Coordinate
      * multiple object coordination
      * multiple camera coordination
  * Release windageARforOSG Version 1.1
    1. for windage library version 1.0

  * Release Version 0.3
    1. Single Natural Feature Tracking is Stable then Version 0.2
      * Add camera pose refinement sequence
    1. Support Multiple object tracking
    1. Support Multiple object single coordinate
  * Release windageARforOSG Version 1.0
    1. Support AR Application for OSG using windageARforOSG
  * Release Version 0.2
    1. Dynamic Link Library to Static Link Library
    1. OpenCV 1.1 to OpenCV 1.2 (OpenCV 2.0 Version has some bug)
    1. Single Natural Feature Tracking is Faster then Version 0.1

### Dependency ###
#### Build ####
  1. Microsoft WindowsXP sp3
  1. Microsoft Visual Studio 2008

#### Windows ####
  1. [OpenCV 2.0](http://sourceforge.net/projects/opencvlibrary/)
  1. [GLUT 3.7](http://www.opengl.org/resources/libraries/glut/)
  * windageARforOSG 1.1
    1. windage v1.0 or later
    1. [OpenSceneGraph 2.8](http://www.openscenegraph.org)
  * windageARforOSG 1.0
    1. windage v0.2 or v0.3
    1. [OpenSceneGraph 2.8](http://www.openscenegraph.org)