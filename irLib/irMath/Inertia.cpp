#include "Inertia.h"

#include <irUtils/Diagnostic.h>

namespace irLib
{
	namespace irMath
	{
		Inertia::Inertia(const Real& Ixx, const Real& Iyy, const Real& Izz, const Real& m)
		{
			static_cast<Matrix6&>(*this) << (Matrix3)Eigen::DiagonalMatrix<Real, 3>(Ixx, Iyy, Izz), Matrix3::Zero(),
				Matrix3::Zero(), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix3& I, const Real& m)
		{
			LOGIF(I.isApprox(I.transpose()), "[ERROR] Inertia construction failed. Inertia must be symmetric.");
			static_cast<Matrix6&>(*this) << I, Matrix3::Zero(),
				Matrix3::Zero(), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix3& I, const Vector3& p, const Real& m)
		{
			LOGIF(I.isApprox(I.transpose()), "[ERROR] Inertia construction failed. Inertia must be symmetric.");
			static_cast<Matrix6&>(*this) << I, -Bracket(p),
				Bracket(p), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Vector6& I, const Vector3& p, const Real& m)
		{
			Matrix3 _I;
			_I(0, 0) = I(0);
			_I(0, 1) = -I(3);
			_I(0, 2) = -I(4);

			_I(1, 0) = -I(3);
			_I(1, 1) = I(1);
			_I(1, 2) = -I(5);

			_I(2, 0) = -I(4);
			_I(2, 1) = -I(5);
			_I(2, 2) = I(2);
			static_cast<Matrix6&>(*this) << _I, -Bracket(p),
				Bracket(p), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix6& I)
		{
			Matrix3 _I = I.block<3, 3>(0, 0);
			Matrix3 _p = I.block<3, 3>(0, 3);
			Matrix3 _m = I.block<3, 3>(3, 3);

			LOGIF(I.isApprox(I.transpose()), "[ERROR] Inertia construction failed. Inertia must be symmetric.");
			LOGIF(_m.isApprox(Matrix3::Identity()*_m(0, 0)), "[ERROR] Inertia construction failed.");

			static_cast<Matrix6&>(*this) = I;
		}

		Inertia::operator Matrix6()
		{
			return static_cast<Matrix6&>(*this);
		}

		Inertia& Inertia::operator = (const Inertia& I)
		{
			static_cast<Matrix6&>(*this) = static_cast<const Matrix6&>(I);
			return *this;
		}

		Inertia Inertia::operator + (const Inertia& I) const
		{
			Inertia result;
			static_cast<Matrix6&>(result) = *static_cast<const Matrix6*>(this) + static_cast<const Matrix6&>(I);
			return result;
		}

		Inertia& Inertia::operator += (const Inertia& I)
		{
			static_cast<Matrix6&>(*this) += static_cast<const Matrix6&>(I);
			return *this;
		}

		Inertia Inertia::operator - (const Inertia& I) const
		{
			Inertia result;
			static_cast<Matrix6&>(result) = *static_cast<const Matrix6*>(this) - static_cast<const Matrix6&>(I);
			return result;
		}

		Inertia& Inertia::operator -= (const Inertia& I)
		{
			static_cast<Matrix6&>(*this) -= static_cast<const Matrix6&>(I);
			return *this;
		}

		Inertia Inertia::operator * (const Real& constant) const
		{
			Inertia result;
			static_cast<Matrix6&>(result) = static_cast<const Matrix6&>(*this) * constant;
			return result;
		}

		Inertia& Inertia::operator *= (const Real& constant)
		{
			static_cast<Matrix6&>(*this) *= constant;
			return *this;
		}

		Inertia Inertia::operator / (const Real& constant) const
		{
			Inertia result;
			static_cast<Matrix6&>(result) = static_cast<const Matrix6&>(*this) / constant;
			return result;
		}

		Inertia& Inertia::operator /= (const Real& constant)
		{
			*static_cast<Matrix6*>(this) /= constant;
			return *this;
		}

		void Inertia::changeFrame(const SE3& T)
		{
			static_cast<Matrix6&>(*this) = SE3::InvAd(T).transpose() * static_cast<Matrix6&>(*this) * SE3::InvAd(T);
		}
	}
}