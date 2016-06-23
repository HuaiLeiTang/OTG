#pragma once

#include "Constant.h"
#include "LieGroup.h"

namespace irLib
{
	namespace irMath
	{
		class Inertia : public Matrix6
		{
		public:
			Inertia() : Matrix6(Matrix6::Identity()) {}
			Inertia(const Real& Ixx, const Real& Iyy, const Real& Izz, const Real& m);
			Inertia(const Real m) : Matrix6(Matrix6::Identity() * m) {}
			Inertia(const Matrix3& I, const Real& m);
			Inertia(const Matrix3& I, const Vector3& p, const Real& m);
			Inertia(const Vector6& I, const Vector3& p, const Real& m);
			Inertia(const Inertia& I) : Matrix6(I) {}
			Inertia(const Matrix6& I);

			~Inertia() {}

			operator Matrix6();

			Inertia& operator = (const Inertia&);
			Inertia operator + (const Inertia&) const;
			Inertia& operator += (const Inertia&);
			Inertia operator - (const Inertia&) const;
			Inertia& operator -= (const Inertia&);
			Inertia operator * (const Real&) const;
			Inertia& operator *= (const Real&);
			Inertia operator / (const Real&) const;
			Inertia& operator /= (const Real&);

			void changeFrame(const SE3& T ///< Transformation from {a} to {b}, T_ba
				);
			static Inertia changeFrame(const SE3& T, const Inertia& G)
			{
				Inertia newG;
				static_cast<Matrix6&>(newG) = static_cast<const Matrix6&>(G);
				newG.changeFrame(T);
				return newG;
			}

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
	}
}