#pragma once

#include <string>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Node>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>

namespace irLib
{
	namespace irRenderer
	{
		class ExtFile
		{
		public:
			ExtFile(const std::string&	fileName, float scale = 1.0)
			{
				_transform = new osg::MatrixTransform;
				_transform->setMatrix(osg::Matrixf::identity());

				_scale = new osg::MatrixTransform;
				_scale->setMatrix(osg::Matrixf::scale(osg::Vec3(scale, scale, scale)));

				_extFile = osgDB::readNodeFile(fileName);

				_material = new osg::Material;
				_material->setColorMode(osg::Material::ColorMode::AMBIENT_AND_DIFFUSE);
				_material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 0.6));
				_material->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 0.1));
				_material->setSpecular(osg::Material::FRONT, osg::Vec4(.5, .5, .5, .5));
				_material->setShininess(osg::Material::FRONT, 10);

				_transform->addChild(_scale);
				_scale->addChild(_extFile);
				_extFile->getOrCreateStateSet()->setAttribute(_material);
			}

			const osg::ref_ptr<osg::MatrixTransform>&	getRoot() const
			{
				return _transform;
			}

			void	setTransform(const osg::Matrix& mat)
			{
				_transform->setMatrix(mat);
			}

			void	setPosition(const osg::Vec3& pos)
			{
				auto mat = _transform->getMatrix();
				mat.setTrans(pos);
				_transform->setMatrix(mat);
			}

			void	setScale(float scale)
			{
				_scale->setMatrix(osg::Matrix::scale(osg::Vec3(scale, scale, scale)));
			}

		protected:
			osg::ref_ptr<osg::Node>				_extFile;
			osg::ref_ptr<osg::Material>			_material;
			osg::ref_ptr<osg::MatrixTransform>	_transform;
			osg::ref_ptr<osg::MatrixTransform>	_scale;
		};
	}
}