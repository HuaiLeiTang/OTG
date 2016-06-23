/*!
*	\file	Constant.h
*	\date	2016.01.20
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr), Keunjun Choi(ckj.monikaru@gmail.com)
*	\brief	Define type and useful constant.
*/

#pragma once

#include <Eigen/Dense>

namespace irLib
{
	namespace irMath
	{
		//typedef float	Real;
		typedef	double	Real;

		typedef Eigen::Matrix<Real, -1, -1>		MatrixX;
		typedef Eigen::Matrix<Real, 3, 3>		Matrix3;
		typedef Eigen::Matrix<Real, 4, 4>		Matrix4;
		typedef Eigen::Matrix<Real, 6, 6>		Matrix6;
		typedef Eigen::Matrix<Real, 6, -1>		Matrix6X;
		typedef Eigen::Matrix<Real, -1, 1>		VectorX;
		typedef Eigen::Matrix<Real, 2, 1>		Vector2;
		typedef Eigen::Matrix<Real, 3, 1>		Vector3;
		typedef Eigen::Matrix<Real, 4, 1>		Vector4;
		typedef Eigen::Matrix<Real, 6, 1>		Vector6;

		typedef Eigen::Matrix<unsigned int, -1, 1>		VectorU;
		typedef Eigen::Matrix<int, -1, 1>				VectorI;

		static const Real	PI_DOUBLE = 6.28318530717958647692;				///< \pi*2
		static const Real	Inv_PI_DOUBLE = 0.15915494309189533576901767873386;	///< \frac{1}{\pi*2}
		static const Real	PI = 3.14159265358979323846;				///< \pi
		static const Real	PI_HALF = 1.57079632679489661923;				///< \frac{\pi}{2}
		static const Real	PI_DIVIDED_BY_SQRT2 = 2.2214414690791831235079404950303;	///< \frac{\pi}{\sqrt{2}}
		static const Real	PI_SQUARE = 9.8696044010893586188344909998762;	///< \pi^2
		static const Real	DEG2RAD = 0.01745329251994329577;				///< rad = DEG2RAD * deg
		static const Real	RAD2DEG = 57.2957795130823208768;				///< deg = RAD2DEG * rad

		static const Real	InverseKinematicsExitCondition = 1e-11;

		static const Real	RealEps = std::numeric_limits<Real>::epsilon();
		static const Real	RealMax = std::numeric_limits<Real>::max();
		static const Real	RealMin = std::numeric_limits<Real>::lowest();
	}
}