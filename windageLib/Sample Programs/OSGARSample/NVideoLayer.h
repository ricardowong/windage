#pragma once

/** *******************************************************************************************************************
 *  CNVideoLayer
 *  @author Kiyoung Kim
 *  last updated: 2008. 7. 14
 *
 *  Usage 
 *  Basically, this is same as osgART\VideoLayer class. 
 ** *******************************************************************************************************************
*/

#ifndef NEW_VIDEOLAYER
#define NEW_VIDEOLAYER

// OSG include
#include "windows.h"
#include <cmath>
#include <osg/Geode>
#include <osg/Node>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Projection>
#include <osg/Texture2D>
#include <osg/Image>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/TextureRectangle>

class Texture2DCallback : public osg::Texture2D::SubloadCallback
{
public:

	/*static*/
	static unsigned int mathNextPowerOf2(unsigned int x)
	{
	#if defined(_MSC_VER) && defined(_X86_)
		unsigned int result = 0;
		_asm   bsr  ecx,x;      // Find the base 2 logarithm
		_asm   inc  ecx;        // Increase it by 1
		_asm   mov  eax,1;
		_asm   shl  eax,cl;     // Find the antilogarithm
		_asm   mov  result,eax;
		return result;          // Return the result
	#else
		return ((unsigned int)(exp2((double)((int)(log2((double)x)) + 1))));
	#endif
	}

	Texture2DCallback(osg::Texture2D* texture)
	{
		_texCoordX = (texture->getImage()->s() / (float)mathNextPowerOf2((unsigned int)texture->getImage()->s()));
		_texCoordY = (texture->getImage()->t() / (float)mathNextPowerOf2((unsigned int)texture->getImage()->t()));

		texture->setTextureSize(mathNextPowerOf2((unsigned int)texture->getImage()->s()),
			mathNextPowerOf2((unsigned int)texture->getImage()->t()));

		texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
		texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
		texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP);
		texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP);
	}

	void load(const osg::Texture2D& texture, osg::State&) const
	  {
		  const osg::Image* _image = texture.getImage();

		  glTexImage2D(GL_TEXTURE_2D, 0, 
			  // hse25: internal texture format gets overwritten by the image format 
			  // we need just the components - ???
			  osg::Image::computeNumComponents(_image->getInternalTextureFormat()), 
			  (float)mathNextPowerOf2((unsigned int)_image->s()), 
			  (float)mathNextPowerOf2((unsigned int)_image->t()), 
			  0, _image->getPixelFormat(), 
			  _image->getDataType(), 0);
	  }

	  void subload(const osg::Texture2D& texture, osg::State&) const
	  {
		  const osg::Image* _image = texture.getImage();

		  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
			  _image->s(), _image->t(), _image->getPixelFormat(), 
			  _image->getDataType(), _image->data());

	  }

	  inline float getTexCoordX() const { return (_texCoordX);};
	  inline float getTexCoordY() const { return (_texCoordY);};

protected:

	float _texCoordX;
	float _texCoordY;

};


class CNVideoLayer: public osg::Group
{
private:
	float m_width;
	float m_height;
	int m_layerDepth;
	float m_alpha;

	osg::ref_ptr<osg::Geometry>			m_geometry;
	
	/* Member variables */
	osg::ref_ptr<osg::MatrixTransform>	m_layerModelViewMatrix;
	osg::ref_ptr<osg::Projection>		m_layerProjectionMatrix;
	osg::ref_ptr<osg::Geode>			m_layerGeode;
	osg::ref_ptr<osg::StateSet>			m_layerStateSet;

	osg::ref_ptr<osg::Image>			m_image;

	/** 
	* Modes how the video is being rendered.
	*/
	enum TextureMode {
		USE_TEXTURE_DEFAULT = 0,	/**< Default texture mode */
		USE_TEXTURE_2D,				/**< Video is being pasted into a 2D texture */
		USE_TEXTURE_RECTANGLE,		/**< Video is being attached to a 2D rectangle */
	};

	/**
	* Mode used to correct camera distortion.
	*/
	enum DistortionCorrectionMode {
		NO_CORRECTION,			/**< No correction is applied to the Texture */
		CAMERA_PARAM_CORRECTION	/**< The texture is being corrected from camera parameters */
	};

	DistortionCorrectionMode m_distortionMode;
	TextureMode m_textureMode;

public:
	
	// VideoLayer
	CNVideoLayer(osg::Image* image /* = 0L*/, int layerD)
	{
		m_image  = image;
		m_width  = image->s();
		m_height = image->t();
		m_layerDepth = layerD;
		m_alpha = -1;
		m_distortionMode = NO_CORRECTION;
		m_textureMode = USE_TEXTURE_2D;
	}

	CNVideoLayer(const CNVideoLayer& videolayer, const osg::CopyOp& copyop)  
	{
	}

	CNVideoLayer();
	~CNVideoLayer();


	/* virtual */
	void setImageSource(osg::Image* image)
	{
		m_width  = (image) ? image->t() : 0;
		m_height = (image) ? image->s() : 0;
	}

	/* virtual */
	void init()
	{
		// add as a child
		this->addChild(buildLayer().get());
	}

	void setTransparency(float alpha) 
	{
		m_alpha=alpha;
		if (alpha<1.0f) //if no transparency, non activate blending op (if already activate, override)
		{
			osg::BlendFunc* blendFunc = new osg::BlendFunc();
			blendFunc->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		
			osg::StateSet* stateset = m_geometry->getStateSet();
		
			stateset->setAttribute(blendFunc);
			stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
			stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		}


		osg::Vec4Array* colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,alpha));
		m_geometry->setColorArray(colors);
		m_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}

	void setLayerDepth(int level)
	{
		m_layerDepth=level;
		m_layerStateSet->setRenderBinDetails(m_layerDepth, "RenderBin");
	}
	
	osg::ref_ptr<osg::Projection> buildLayer() 
	{
		m_layerProjectionMatrix = new osg::Projection(osg::Matrix::ortho2D(0, m_width, 0, m_height));
		m_layerModelViewMatrix  = new osg::MatrixTransform();
		m_layerModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		m_layerProjectionMatrix->addChild(m_layerModelViewMatrix.get());

		osg::Group* layerGroup = new osg::Group();
		m_layerModelViewMatrix->addChild(layerGroup);

		m_layerStateSet = new osg::StateSet();
		layerGroup->setStateSet(m_layerStateSet.get());

		setLayerDepth(m_layerDepth);
		layerGroup->getOrCreateStateSet()->setAttribute(new osg::Depth(osg::Depth::ALWAYS, 1.0f, 1.0f));
		layerGroup->addChild(buildLayerGeometry().get());

		return m_layerProjectionMatrix;
	}

	osg::ref_ptr<osg::Geode> buildLayerGeometry() 
	{
		osg::Texture* _texture = 0L;

		float maxU = 1.0f, maxV = 1.0f;

		switch (m_textureMode) 
		{
		case USE_TEXTURE_RECTANGLE:
			
			osg::notify() << "osgART::GenericVideoObject() using TextureRectangle" << std::endl;

			_texture = new osg::TextureRectangle(this->m_image.get());
			
			maxU = m_image->s();
			maxV = m_image->t();
			
			break;

		case USE_TEXTURE_DEFAULT:
		case USE_TEXTURE_2D:
		default:

			osg::notify() << "osgART::GenericVideoObject() using Texture2D" << std::endl;

			_texture = new osg::Texture2D(this->m_image.get());

			Texture2DCallback *_cb = new Texture2DCallback(dynamic_cast<osg::Texture2D*>(_texture));

			maxU = _cb->getTexCoordX();
			maxV = _cb->getTexCoordY();

			dynamic_cast<osg::Texture2D*>(_texture)->setSubloadCallback(_cb);
		}


		_texture->setDataVariance(osg::Object::DYNAMIC);

		m_layerGeode = new osg::Geode();

		m_geometry = new osg::Geometry();
		
		osg::Vec3Array* coords = new osg::Vec3Array();
		m_geometry->setVertexArray(coords);

		osg::Vec2Array* tcoords = new osg::Vec2Array();
		m_geometry->setTexCoordArray(0, tcoords);

		switch (m_distortionMode) 
		{
			case CAMERA_PARAM_CORRECTION:
			{
				
			}

			default:
				osg::notify() << "osgART::VideoLayer::buildLayerGeometry()"
					"Undefined distortion mode" << std::endl;
			case NO_CORRECTION:
			{

				coords->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
				coords->push_back(osg::Vec3(m_width, 0.0f, 0.0f));
				coords->push_back(osg::Vec3(m_width, m_height, 0.0f));
				coords->push_back(osg::Vec3(0.0f, m_height, 0.0f));

				tcoords->push_back(osg::Vec2(0.0f, maxV));
				tcoords->push_back(osg::Vec2(maxU, maxV));
				tcoords->push_back(osg::Vec2(maxU, 0.0f));
				tcoords->push_back(osg::Vec2(0.0f, 0.0f));

				m_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));

				break;
			}
		}
	    
		m_geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);
		m_geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
		m_layerGeode->addDrawable(m_geometry.get());
		return m_layerGeode;
	}
};

#endif