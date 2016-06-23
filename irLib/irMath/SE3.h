/*!
*	\file	SE3.h
*	\date	2016.01.23
*	\author	Keunjun Choi(ckj.monikaru@gmail.com)
*	\brief	SE3 class header file
*/

#pragma once

#include <iostream>
#include "Constant.h"
#include "SO3.h"

namespace irLib
{
	namespace irMath
	{
		class SE3;
		typedef Vector6 se3;
		typedef Vector6 dse3;

		/*!
		*	\class SE3
		*/
		class SE3
		{
		public:
			SE3(const SE3& T);
			SE3(const Vector3& p = (Vector3::Zero())) : _R(), _p(p) {}
			SE3(const SO3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			SE3(const Matrix3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			SE3(const Matrix4& T);

			~SE3() {}

			SE3& operator = (const SE3&);
			SE3& operator = (const Matrix4&);
			SE3 operator * (const SE3&) const;
			SE3& operator *= (const SE3&);
			const Matrix4 matrix() const;

			void setRotation(const SO3& R);
			void setRotation(const Matrix3& M);
			void setPosition(const Vector3& p);
			const SO3& getRotation() const;
			const Vector3& getPosition() const;

			friend std::ostream& operator << (std::ostream&, const SE3&);

			SE3 inverse() const;

			static SE3 Exp(const se3& S, Real angle = (1.0));
			static SE3 Exp(so3 w, Vector3 v, Real angle = (1.0));

			static se3 Log(const SE3&);

			static Matrix6 Ad(const SE3& T);
			static se3 Ad(const SE3& T, const se3& S);
			static Matrix6 InvAd(const SE3& T);
			static se3 InvAd(const SE3& T, const se3& S);

			static Matrix6 ad(const se3& S);
			static Matrix6 adTranspose(const se3& S);

			static se3 ad(const se3& S1, const se3& S2);
			static se3 adTranspose(const se3& S1, const se3& S2);

		private:
			SO3 _R;
			Vector3 _p;
		};

		static Matrix4 Bracket(const se3& S)
		{
			Matrix4 result;

			result(0, 0) = 0;
			result(0, 1) = -S(2);
			result(0, 2) = S(1);

			result(1, 0) = S(2);
			result(1, 1) = 0;
			result(1, 2) = -S(0);

			result(2, 0) = -S(1);
			result(2, 1) = S(0);
			result(2, 2) = 0;

			result(0, 3) = S(3);
			result(1, 3) = S(4);
			result(2, 3) = S(5);

			result(3, 0) = 0;
			result(3, 1) = 0;
			result(3, 2) = 0;
			result(3, 3) = 0;

			return result;
		}
	}
}