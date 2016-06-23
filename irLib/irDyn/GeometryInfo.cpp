#include "GeometryInfo.h"

#include <io.h>

using namespace std;
using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		GeometryInfo::~GeometryInfo() {}

		void Box::setDimension(const Real& width, const Real& depth, const Real& height)
		{
			_width = width;
			_depth = depth;
			_height = height;
		}

		void Box::setDimension(const Vector3& config)
		{
			_width = config(0);
			_depth = config(1);
			_height = config(2);
		}

		void Box::setCube(const Real& length)
		{
			_width = _depth = _height = length;
		}

		Vector3 Box::getDimension() const
		{
			return Vector3(_width, _depth, _height);
		}

		GeometryInfoPtr Box::copy() const
		{
			return GeometryInfoPtr(new Box(*this));
		}

		void Sphere::setRadius(const Real& radius)
		{
			_radius = radius;
		}

		const Real& Sphere::getRadius() const
		{
			return _radius;
		}

		GeometryInfoPtr Sphere::copy() const
		{
			return GeometryInfoPtr(new Sphere(*this));
		}

		void Capsule::setDimension(const Real& radius, const Real& height)
		{
			_radius = radius;
			_height = height;
		}

		void Capsule::setDimension(const Vector2& config)
		{
			_radius = config(0);
			_height = config(1);
		}

		Vector2 Capsule::getDimension() const
		{
			return Vector2(_radius, _height);
		}

		GeometryInfoPtr Capsule::copy() const
		{
			return GeometryInfoPtr(new Capsule(*this));
		}

		void Cylinder::setDimension(const Real& radius, const Real& height)
		{
			_radius = radius;
			_height = height;
		}

		void Cylinder::setDimension(const Vector2& config)
		{
			_radius = config(0);
			_height = config(1);
		}

		Vector2 Cylinder::getDimension() const
		{
			return Vector2(_radius, _height);
		}

		GeometryInfoPtr Cylinder::copy() const
		{
			return GeometryInfoPtr(new Cylinder(*this));
		}

		bool Mesh::isValid() const
		{
			if (_access(_url.c_str(), 0) == 0)
			{
				return true;
			}
			return false;
		}

		GeometryInfoPtr Mesh::copy() const
		{
			return GeometryInfoPtr(new Mesh(*this));
		}

		void UserModel::addList(const GeometryInfoPtr& shape, const SE3& T)
		{
			_GeometryList.push_back(std::pair< GeometryInfoPtr, SE3 >(shape, T));
		}

		GeometryInfoPtr UserModel::copy() const
		{
			UserModel *clone = new UserModel();
			clone->setType(this->getType());
			clone->setColor(this->getColor());
			clone->setFrame(this->getTransform());
			for (list< pair <GeometryInfoPtr, SE3> >::const_iterator iterPos = _GeometryList.begin(); iterPos != _GeometryList.end(); iterPos++)
			{
				clone->addList(iterPos->first->copy(), iterPos->second);
			}
			return GeometryInfoPtr(clone);
		}
	}
}