#include <iostream>
#include "OSG_NodeVisitor.h"

namespace irLib
{
	namespace irRenderer
	{
		OSG_NodeVisitor::OSG_NodeVisitor()
			: NodeVisitor(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN)
		{
			_color.set(1.0, 1.0, 1.0, 1.0);
			_colorArray = new osg::Vec4Array;
			_colorArray->push_back(_color);
			_colorArray->setBinding(osg::Array::BIND_OVERALL);
		}

		void OSG_NodeVisitor::apply(osg::Node & node)
		{
			traverse(node);
		}

		void OSG_NodeVisitor::apply(osg::Geode & geode)
		{
			osg::StateSet *state = NULL;
			unsigned int    vertNum = 0;

			//  We need to iterate through all the drawables check if
			//  the contain any geometry that we will need to process
			unsigned int numGeoms = geode.getNumDrawables();
			for (unsigned int geodeIdx = 0; geodeIdx < numGeoms; geodeIdx++) {

				// Use 'asGeometry' as its supposed to be faster than a dynamic_cast
				// every little saving counts
				osg::Geometry *curGeom = geode.getDrawable(geodeIdx)->asGeometry();

				// Only process if the drawable is geometry
				if (curGeom) {
					osg::Vec4Array *colorArrays = dynamic_cast<osg::Vec4Array *>(curGeom->getColorArray());

					if (colorArrays) {
						for (unsigned int i = 0; i < colorArrays->size(); i++)
						{
							osg::Vec4 *color = &colorArrays->operator [](i);
							// could also use *color = m_color
							color->set(_color._v[0], _color._v[1], _color._v[2], _color._v[3]);
						}
					}
					else
					{
						curGeom->setColorArray(_colorArray.get());
						//curGeom->setColorArray(_colorArray.get());
						//curGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
					}
				}

			}
		}

		void OSG_NodeVisitor::setColor(const float r, const float g, const float b, const float a)
		{
			osg::Vec4 *c = &_colorArray->operator [](0);
			_color.set(r, g, b, a);
			*c = _color;
		}
	}
}