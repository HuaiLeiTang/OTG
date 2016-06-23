#include "Link.h"

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		void Link::setInertia(const Inertia & I)
		{
			_inertia = I;
		}

		void Link::addDrawingGeomtryInfo(const GeometryInfoPtr& visual)
		{
			_drawingGeometryInfo.push_back(visual);
		}

		void Link::addCollisionGeometryInfo(const GeometryInfoPtr& collision)
		{
			_collisionGeometryInfo.push_back(collision);
		}

		const Inertia & Link::getInertia() const
		{
			return _inertia;
		}

		const std::vector<GeometryInfoPtr>& Link::getDrawingGeometryInfo() const
		{
			return _drawingGeometryInfo;
		}

		const std::vector<GeometryInfoPtr>& Link::getCollisionGeometryInfo() const
		{
			return _collisionGeometryInfo;
		}

		LinkPtr Link::copy() const
		{
			LinkPtr Link_clone(new Link(_inertia));
			for (unsigned int i = 0; i < _drawingGeometryInfo.size(); i++)
				Link_clone->addDrawingGeomtryInfo(_drawingGeometryInfo[i]->copy());
			for (unsigned int i = 0; i < _collisionGeometryInfo.size(); i++)
				Link_clone->addCollisionGeometryInfo(_collisionGeometryInfo[i]->copy());
			return Link_clone;
		}
	}
}