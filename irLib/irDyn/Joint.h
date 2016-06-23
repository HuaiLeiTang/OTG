/*!
*	\file	Joint.h
*	\date	2016.01.22
*	\author	Youngsuk (crazyhys@gmail.com)
*	\brief	Join class
*         description of the industrial robot motors
*/

#pragma once

#include <string>
#include <memory>

#include <Eigen\Dense>
#include <irMath\LieGroup.h>
#include <irMath\Constant.h>

namespace irLib
{
	namespace irDyn
	{
		class Joint;
		typedef std::shared_ptr<Joint> JointPtr;

		class Joint
		{
		private:
			/*!
			* \brief Joint class member variable
			*/

			// Motor Properties
			irMath::Real _rotorinertia;
			irMath::Real _resistance;
			irMath::Real _inductance;
			irMath::Real _gearRatio;
			irMath::Real _motorConstant;
			irMath::Real _backEMFConstant;

			irMath::Real _springConstant;
			irMath::Real _damperConstant;
			irMath::Real _frictionConstant;

			// Motor limit Values	
			irMath::Real _LimitPosLower;
			irMath::Real _LimitPosUpper;
			irMath::Real _LimitVelLower;
			irMath::Real _LimitVelUpper;
			irMath::Real _LimitAccLower;
			irMath::Real _LimitAccUpper;
			irMath::Real _LimitJerkLower;
			irMath::Real _LimitJerkUpper;
			irMath::Real _LimitTorqueLower;
			irMath::Real _LimitTorqueUpper;

			// Joint member values
			irMath::Vector6 _axis;

		public:
			/*!
			* \brief Joint class member functions
			*/

			// constructor & destructor
			Joint();
			Joint(const irMath::Vector6& axis);
			~Joint();

			// set-function
			void setRotorInertia(const irMath::Real RI);
			void setResistance(const irMath::Real R);
			void setInductance(const irMath::Real In);
			void setGearRatio(const irMath::Real Gear);
			void setMotorConstant(const irMath::Real MC);
			void setBackEMFConstant(const irMath::Real BEMFC);
			void setSpringConstant(const irMath::Real Spring);
			void setDamperConstant(const irMath::Real Damper);
			void setFrictionConstant(const irMath::Real Friction);

			bool setLimitPos(const irMath::Real lower, const irMath::Real upper);
			bool setLimitVel(const irMath::Real lower, const irMath::Real upper);
			bool setLimitAcc(const irMath::Real lower, const irMath::Real upper);
			bool setLimitJerk(const irMath::Real lower, const irMath::Real upper);
			bool setLimitTorque(const irMath::Real lower, const irMath::Real upper);

			void setAxis(const irMath::Vector3& axis); // when only setting in angular velocity
			void setAxis(const irMath::Vector6& axis);

			// get-function
			const irMath::Real getRotorInertia() const;
			const irMath::Real getResistance() const;
			const irMath::Real getInductance() const;
			const irMath::Real getGearRatio() const;
			const irMath::Real getMotorConstant() const;
			const irMath::Real getBackEMFConstant() const;
			const irMath::Real getSpringConstant() const;
			const irMath::Real getDamperConstant() const;
			const irMath::Real getFrictionConstant() const;

			const irMath::Real getLimitPosLower() const;
			const irMath::Real getLimitPosUpper() const;
			const irMath::Real getLimitVelLower() const;
			const irMath::Real getLimitVelUpper() const;
			const irMath::Real getLimitAccLower() const;
			const irMath::Real getLimitAccUpper() const;
			const irMath::Real getLimitJerkLower() const;
			const irMath::Real getLimitJerkUpper() const;
			const irMath::Real getLimitTorqueLower() const;
			const irMath::Real getLimitTorqueUpper() const;

			const irMath::Vector6& getAxis() const;

			// deep-copy
			JointPtr copy() const;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};
	}
}