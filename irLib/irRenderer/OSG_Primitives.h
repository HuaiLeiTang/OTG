#pragma once

#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Point>

namespace irLib
{
	namespace irRenderer
	{
		class Primitives
		{
		public:
			Primitives(osg::Vec4 color = osg::Vec4(0.0, 0.0, 0.0, 1.0))
			{
				_geometry = new osg::Geometry();
				_points = new osg::Vec3Array;
				_color = new osg::Vec4Array;
				_color->push_back(color);
				_color->setBinding(osg::Array::BIND_OVERALL);
				_geometry->setVertexArray(_points);
				_geometry->setColorArray(_color);
			}
			virtual ~Primitives() {}


			void push_back(const osg::Vec3&	point)
			{
				_points->push_back(point);
				_drawArrays->setFirst(0);
				_drawArrays->setCount(_points->size());
				_geometry->setVertexArray(_points);
			}

			void setColor(float r, float g, float b, float a = 1.0)
			{
				(*_color)[0][0] = r;
				(*_color)[0][1] = g;
				(*_color)[0][2] = b;
				(*_color)[0][3] = a;
			}

			const osg::ref_ptr<osg::Geometry>&	getGeometry() const
			{
				return _geometry;
			}

		protected:
			osg::ref_ptr<osg::Geometry>			_geometry;
			osg::ref_ptr<osg::Vec3Array>		_points;
			osg::ref_ptr<osg::Vec4Array>		_color;
			osg::ref_ptr<osg::DrawArrays>		_drawArrays;
		};


		class Line : public Primitives
		{
		public:
			Line(float lineWidth = 2, osg::Vec4 color = osg::Vec4(0.0, 0.0, 0.0, 1.0))
				: Primitives(color)
			{
				_lineWidth = new osg::LineWidth(lineWidth);
				_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
				_geometry->getOrCreateStateSet()->setAttributeAndModes(_lineWidth, osg::StateAttribute::ON);
				_geometry->addPrimitiveSet(_drawArrays);

			}

			virtual ~Line() {}

			void setWidth(float w) { _lineWidth->setWidth(w); }
		protected:
			osg::ref_ptr<osg::LineWidth>		_lineWidth;
		};

		class Points :public Primitives
		{
		public:
			Points(float size = 3, osg::Vec4 color = osg::Vec4(0.0, 0.0, 0.0, 1.0))
				: Primitives(color)
			{
				_point = new osg::Point(size);
				_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
				_geometry->getOrCreateStateSet()->setAttribute(_point);
				_geometry->getOrCreateStateSet()->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);
				_geometry->addPrimitiveSet(_drawArrays);
			}

			void setSize(float d) { _point->setSize(d); }

		protected:
			osg::ref_ptr<osg::Point>	_point;
		};
	}
}