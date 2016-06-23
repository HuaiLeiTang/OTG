/*!
*	\file	Link.h
*	\date	2016.01.22
*	\author	Youngsuk (crazyhys@gmail.com)
*	\brief	Link class
*/

#pragma once

#include <vector>
#include <memory>

#include <irUtils\Diagnostic.h>
#include <irMath\LieGroup.h>
#include <irMath\Inertia.h>

#include "GeometryInfo.h"


namespace irLib
{
	namespace irDyn
	{
		class Link;

		typedef std::shared_ptr<Link> LinkPtr;

		class Link
		{
		private:
			/*!
			* \brief Link class member variable
			*/
			irMath::Inertia _inertia;
			std::vector<GeometryInfoPtr> _drawingGeometryInfo;
			std::vector<GeometryInfoPtr> _collisionGeometryInfo;

		public:
			/*!
			* \brief Link class member functions
			*/

			// constructor & destructor
			Link(const irMath::Inertia& I = irMath::Inertia(),
				std::vector<GeometryInfoPtr>& visual = std::vector<GeometryInfoPtr>(),
				std::vector<GeometryInfoPtr>& collision = std::vector<GeometryInfoPtr>()
				) : _inertia(I), _drawingGeometryInfo(visual), _collisionGeometryInfo(collision) {}

			// set-function
			void setInertia(const irMath::Inertia& I);
			void addDrawingGeomtryInfo(const GeometryInfoPtr& visual);
			void addCollisionGeometryInfo(const GeometryInfoPtr& collision);

			// get-function
			const irMath::Inertia& getInertia() const;
			const std::vector<GeometryInfoPtr>& getDrawingGeometryInfo() const;
			const std::vector<GeometryInfoPtr>& getCollisionGeometryInfo() const;

			// deep-copy
			LinkPtr copy() const;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};
	}
}