#pragma once

#include <iostream>
#include "Constant.h"
#include "Common.h"

namespace irLib
{
	namespace irMath
	{
		class SO3;
		typedef Vector3 so3;
		typedef Vector3 dso3;

		class SO3
		{
			friend class SE3;

		public:
			SO3() : _e(Matrix3::Identity()) {}

			SO3(const SO3& R) : _e(R._e) {}

			SO3(const Matrix3& R);

			~SO3() {}

			SO3& operator = (const SO3&);
			SO3& operator = (const Matrix3&);
			SO3 operator * (const SO3&) const;
			SO3& operator *= (const SO3&);
			const Matrix3& matrix() const;

			Vector3 getX() const;
			Vector3 getY() const;
			Vector3 getZ() const;

			friend std::ostream& operator << (std::ostream&, const SO3&);

			SO3 inverse() const;

			static SO3 RotX(const Real angle);
			static SO3 RotY(const Real angle);
			static SO3 RotZ(const Real angle);
			static SO3 EulerZYX(const Real angle1, const Real angle2, const Real angle3);
			static SO3 EulerZYZ(const Real angle1, const Real angle2, const Real angle3);

			static Vector3 invEulerZYX(const SO3& R);
			static Vector3 invEulerZYZ(const SO3& R);

			static SO3 Exp(so3 w, Real angle = (1.0));

			static so3 Log(const SO3& R);

			static SO3 Projection(const Matrix3&);

		private:
			Matrix3 _e;
		};

		static Matrix3 Bracket(const so3 w)
		{
			Matrix3 result;

			result(0, 0) = 0;
			result(0, 1) = -w(2);
			result(0, 2) = w(1);

			result(1, 0) = w(2);
			result(1, 1) = 0;
			result(1, 2) = -w(0);

			result(2, 0) = -w(1);
			result(2, 1) = w(0);
			result(2, 2) = 0;

			return result;
		}
	}
}