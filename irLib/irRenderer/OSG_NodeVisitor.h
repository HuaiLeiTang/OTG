#pragma once

#include <osg/array>
#include <osg/geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/Vec4>


namespace irLib
{
	namespace irRenderer
	{
		class OSG_NodeVisitor : public osg::NodeVisitor
		{
		public:
			OSG_NodeVisitor();

			virtual ~OSG_NodeVisitor() {}

			virtual void apply(osg::Node &node);
			virtual void apply(osg::Geode &geode);

			void setColor(const float r, const float g, const float b, const float a = 1.0f);

		private:

			osg::Vec4 _color;
			osg::ref_ptr<osg::Vec4Array> _colorArray;
		};
	}
}